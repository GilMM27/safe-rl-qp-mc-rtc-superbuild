#pragma once
#include "SmplhDistanceSnapshot.h"

#include <fcl/fcl.h>
#include <mujoco/mujoco.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <condition_variable>
#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace mc_mujoco
{
using SmplhBVH = fcl::BVHModel<fcl::OBBRSSd>;
using SmplhCollisionGeometry = fcl::CollisionGeometryd;

struct SmplhRobotGeom
{
  std::string name;
  std::shared_ptr<SmplhCollisionGeometry> geometry;
};

struct SmplhRobotPose
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

struct SmplhQueryInput
{
  std::uint64_t sequence = 0;
  std::uint32_t frame = 0;
  double time = 0;
  std::vector<Eigen::Vector3d> vertices;
  std::vector<SmplhRobotPose> robot_poses;
};

class SmplhDistanceWorker
{
  struct Region
  {
    std::string name;
    std::vector<uint32_t> vertexIds;
    std::vector<fcl::Triangle> triangles;
    std::vector<fcl::Vector3d> vertices;
    fcl::AABBd bounds;
  };
  std::vector<Region> regions_;
  std::vector<std::shared_ptr<SmplhBVH>> humanModels_;
  std::uint32_t humanModelFrame_ = UINT32_MAX;
  std::vector<SmplhRobotGeom> robots_;
  std::vector<int> robotIds_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::unique_ptr<SmplhQueryInput> pending_;
  SmplhDistanceSnapshot completed_;
  bool stopping_ = false;
  std::thread thread_;

  static std::shared_ptr<SmplhCollisionGeometry> robotGeometry(const mjModel * m, int id)
  {
    const auto * s = m->geom_size + 3 * id;
    switch(m->geom_type[id])
    {
      case mjGEOM_SPHERE: return std::make_shared<fcl::Sphered>(s[0]);
      case mjGEOM_CAPSULE: return std::make_shared<fcl::Capsuled>(s[0], 2 * s[1]);
      case mjGEOM_CYLINDER: return std::make_shared<fcl::Cylinderd>(s[0], 2 * s[1]);
      case mjGEOM_BOX: return std::make_shared<fcl::Boxd>(2 * s[0], 2 * s[1], 2 * s[2]);
      case mjGEOM_MESH:
      {
        int mesh = m->geom_dataid[id];
        int va = m->mesh_vertadr[mesh], nv = m->mesh_vertnum[mesh];
        int fa = m->mesh_faceadr[mesh], nf = m->mesh_facenum[mesh];
        std::vector<fcl::Vector3d> vertices; vertices.reserve(nv);
        for(int i = 0; i < nv; ++i) vertices.emplace_back(m->mesh_vert[3*(va+i)],m->mesh_vert[3*(va+i)+1],m->mesh_vert[3*(va+i)+2]);
        std::vector<fcl::Triangle> faces; faces.reserve(nf);
        for(int i = 0; i < nf; ++i) faces.emplace_back(m->mesh_face[3*(fa+i)],m->mesh_face[3*(fa+i)+1],m->mesh_face[3*(fa+i)+2]);
        auto bvh = std::make_shared<SmplhBVH>();
        bvh->beginModel(nf,nv); bvh->addSubModel(vertices,faces); bvh->endModel(); return bvh;
      }
      default: throw std::runtime_error("Unsupported robot distance geom type");
    }
  }

  SmplhDistanceSnapshot query(const SmplhQueryInput & input)
  {
    const auto preparationStart = std::chrono::steady_clock::now();
    SmplhDistanceSnapshot snapshot;
    if(input.frame != humanModelFrame_)
    {
      for(size_t i=0;i<regions_.size();++i)
      {
        auto & region = regions_[i];
        for(size_t v=0; v<region.vertexIds.size(); ++v)
          region.vertices[v] = input.vertices.at(region.vertexIds[v]);
        region.bounds = fcl::AABBd(region.vertices.front());
        for(const auto & vertex : region.vertices) region.bounds += vertex;
        if(i == humanModels_.size())
        {
          auto model=std::make_shared<SmplhBVH>();
          model->beginModel(static_cast<int>(region.triangles.size()),static_cast<int>(region.vertices.size()));
          model->addSubModel(region.vertices,region.triangles); model->endModel(); humanModels_.push_back(std::move(model));
        }
        else
        {
          // These are instantaneous queries, not continuous collision checks.
          // Replacement refits bounds to this pose only; update would also
          // include the previous pose and weaken pruning across motion frames.
          auto & model=*humanModels_[i];
          model.beginReplaceModel();
          model.replaceSubModel(region.vertices);
          // Fit each node directly from its current primitives. FCL 0.7.0's
          // bottom-up OBBRSS merging disagreed with fresh BVHs on deforming
          // meshes and caused seconds-long distance traversals in replay tests.
          model.endReplaceModel(/* refit = */ true, /* bottomup = */ false);
        }
      }
      humanModelFrame_=input.frame;
    }
    snapshot.ready = true; snapshot.sequence = input.sequence; snapshot.motion_frame = input.frame;
    snapshot.sample_simulation_time = input.time;
    // Build current-pose robot objects once, rather than once per region.
    std::vector<std::unique_ptr<fcl::CollisionObjectd>> robotObjects;
    for(size_t i = 0; i < robots_.size(); ++i)
    {
      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      transform.linear() = input.robot_poses[i].rotation;
      transform.translation() = input.robot_poses[i].translation;
      robotObjects.emplace_back(std::make_unique<fcl::CollisionObjectd>(robots_[i].geometry, transform));
    }
    snapshot.total_pairs = regions_.size() * robots_.size();
    snapshot.preparation_duration_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now()-preparationStart).count();
    for(size_t regionIndex=0; regionIndex<regions_.size(); ++regionIndex)
    {
      const auto & region=regions_[regionIndex];
      fcl::CollisionObjectd human(humanModels_[regionIndex]);
      std::vector<std::pair<double, size_t>> candidates;
      for(size_t i = 0; i < robots_.size(); ++i)
        candidates.emplace_back(region.bounds.distance(robotObjects[i]->getAABB()), i);
      std::sort(candidates.begin(), candidates.end());
      bool have = false; SmplhRegionDistance best; double bestDistance = std::numeric_limits<double>::infinity();
      size_t bestIndex = robots_.size();
      for(const auto & entry : candidates)
      {
        // Small tolerance keeps borderline floating-point comparisons conservative.
        if(have && entry.first > bestDistance + 1e-9) break;
        const size_t i = entry.second;
        const auto & robot = robots_[i];
        auto & object = *robotObjects[i];
        ++snapshot.queried_pairs;
        fcl::CollisionRequestd collisionRequest; fcl::CollisionResultd collisionResult;
        const auto collisionStart = std::chrono::steady_clock::now();
        bool intersecting = fcl::collide(&human, &object, collisionRequest, collisionResult) > 0;
        snapshot.collision_duration_seconds +=
            std::chrono::duration<double>(std::chrono::steady_clock::now()-collisionStart).count();
        SmplhRegionDistance candidate; candidate.region = region.name; candidate.robot_geometry = robot.name;
        if(intersecting) { candidate.distance = 0; candidate.intersecting = true; }
        else
        {
          fcl::DistanceRequestd request(true); fcl::DistanceResultd result;
          const auto distanceStart = std::chrono::steady_clock::now();
          candidate.distance = fcl::distance(&human, &object, request, result);
          snapshot.distance_duration_seconds +=
              std::chrono::duration<double>(std::chrono::steady_clock::now()-distanceStart).count();
          if(!std::isfinite(candidate.distance) || candidate.distance < 0) continue;
          auto ph = result.nearest_points[0], pr = result.nearest_points[1];
          candidate.human_point = {ph.x(),ph.y(),ph.z()}; candidate.robot_point = {pr.x(),pr.y(),pr.z()};
          candidate.closest_points_valid = true;
          Eigen::Vector3d delta = pr - ph;
          if(delta.norm() > 1e-12) { delta.normalize(); candidate.direction = {delta.x(),delta.y(),delta.z()}; candidate.direction_valid = true; }
          if(result.b1 >= 0 && static_cast<size_t>(result.b1) < region.triangles.size())
          {
            const auto & face = region.triangles[result.b1];
            Eigen::Vector3d n = (region.vertices[face[1]] - region.vertices[face[0]]).cross(region.vertices[face[2]] - region.vertices[face[0]]);
            if(n.norm() > 1e-12) { n.normalize(); candidate.human_normal = {n.x(),n.y(),n.z()}; candidate.human_normal_valid = true; }
          }
        }
        if(!have || candidate.distance < bestDistance || (candidate.distance == bestDistance && i < bestIndex))
        { have=true; bestDistance=candidate.distance; bestIndex=i; best=std::move(candidate); }
        if(bestDistance == 0) break;
      }
      if(have) snapshot.regions.push_back(std::move(best));
    }
    return snapshot;
  }

  void run()
  {
    while(true)
    {
      std::unique_ptr<SmplhQueryInput> input;
      { std::unique_lock<std::mutex> lock(mutex_); cv_.wait(lock,[&]{return stopping_ || pending_;});
        if(stopping_) return; input=std::move(pending_); }
      SmplhDistanceSnapshot result;
      const auto start = std::chrono::steady_clock::now();
      try { result=query(*input); }
      catch(const std::exception & e)
      {
        result.sequence=input->sequence; result.motion_frame=input->frame;
        result.sample_simulation_time=input->time; result.error=e.what();
      }
      result.worker_duration_seconds =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      { std::lock_guard<std::mutex> lock(mutex_); if(result.sequence >= completed_.sequence) completed_=std::move(result); }
    }
  }

public:
  SmplhDistanceWorker() = default;
  SmplhDistanceWorker(const SmplhDistanceWorker &) = delete;
  ~SmplhDistanceWorker() { {std::lock_guard<std::mutex> lock(mutex_); stopping_=true;} cv_.notify_one(); if(thread_.joinable()) thread_.join(); }
  void configure(const mjModel * m, const std::vector<std::vector<uint32_t>> & regionFaces,
                 const std::vector<std::string> & regionNames, const uint32_t * faces,
                 const char * prefix="kinova_", int group=2)
  {
    if(regionFaces.size()!=regionNames.size()) throw std::runtime_error("SMPL-H region metadata mismatch");
    for(size_t r=0;r<regionNames.size();++r)
    {
      Region region; region.name=regionNames[r];
      std::unordered_map<uint32_t, uint32_t> localIds;
      for(uint32_t faceId:regionFaces[r])
      {
        std::array<uint32_t,3> local;
        for(size_t corner=0; corner<3; ++corner)
        {
          const uint32_t global = faces[3*faceId+corner];
          auto entry = localIds.emplace(global, static_cast<uint32_t>(region.vertexIds.size()));
          if(entry.second) region.vertexIds.push_back(global);
          local[corner] = entry.first->second;
        }
        region.triangles.emplace_back(local[0],local[1],local[2]);
      }
      if(region.triangles.empty()) throw std::runtime_error("Empty SMPL-H region: "+region.name);
      region.vertices.resize(region.vertexIds.size());
      regions_.push_back(std::move(region));
    }
    std::string wanted=prefix?prefix:"";
    for(int i=0;i<m->ngeom;++i)
    {
      const char * body=mj_id2name(m,mjOBJ_BODY,m->geom_bodyid[i]);
      if(!body || m->geom_group[i]!=group || (!wanted.empty() && std::string(body).compare(0,wanted.size(),wanted)!=0)) continue;
      const char * geom=mj_id2name(m,mjOBJ_GEOM,i);
      robots_.push_back({geom?geom:std::to_string(i),robotGeometry(m,i)});
      robotIds_.push_back(i);
    }
    if(robots_.empty()) throw std::runtime_error("No robot geometries match SMPLH_ROBOT_PREFIX/GROUP");
    thread_=std::thread(&SmplhDistanceWorker::run,this);
  }
  void submit(std::uint64_t seq,uint32_t frame,double time,const float * vertices,uint32_t nvertices,
              const mjData * data,const mjModel * model,int humanSurfaceGeom)
  {
    auto input=std::make_unique<SmplhQueryInput>(); input->sequence=seq; input->frame=frame; input->time=time;
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> humanRotation(data->geom_xmat+9*humanSurfaceGeom);
    Eigen::Map<const Eigen::Vector3d> humanTranslation(data->geom_xpos+3*humanSurfaceGeom);
    input->vertices.reserve(nvertices);
    for(uint32_t i=0;i<nvertices;++i)
    {
      Eigen::Vector3d local(vertices[3*i],vertices[3*i+1],vertices[3*i+2]);
      input->vertices.emplace_back(humanRotation * local + humanTranslation);
    }
    input->robot_poses.reserve(robots_.size());
    for(size_t j=0;j<robots_.size();++j)
    {
      int id=robotIds_[j]; SmplhRobotPose pose;
      Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> rotation(data->geom_xmat+9*id);
      pose.rotation=rotation; pose.translation=Eigen::Map<const Eigen::Vector3d>(data->geom_xpos+3*id);
      input->robot_poses.push_back(pose);
    }
    {std::lock_guard<std::mutex> lock(mutex_); pending_=std::move(input);} cv_.notify_one();
  }
  SmplhDistanceSnapshot latest() const
  {
    auto * self=const_cast<SmplhDistanceWorker *>(this);
    std::lock_guard<std::mutex> lock(self->mutex_); return self->completed_;
  }
};
} // namespace mc_mujoco
