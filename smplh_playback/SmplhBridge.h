#pragma once
// Linux-only, opt-in local bridge. Simulation time and geom transforms use doubles;
// frame payloads use float32 because MuJoCo stores mesh assets as floats.
#include <mujoco/mujoco.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

class SmplhBridge
{
  int fd_ = -1;
  bool checked_ = false;
  int mesh_ = -1;
  uint32_t frame_ = UINT32_MAX;
  bool mesh_dirty_ = false;
  std::vector<int> joints_;
  std::vector<float> response_;
  void transfer(void * p, size_t bytes, bool sendData)
  {
    auto * c = static_cast<char *>(p);
    while(bytes)
    {
      auto n = sendData ? ::send(fd_, c, bytes, MSG_NOSIGNAL) : ::recv(fd_, c, bytes, 0);
      if(n <= 0) throw std::runtime_error("SMPL-H worker disconnected or timed out");
      c += n; bytes -= static_cast<size_t>(n);
    }
  }
  int find(const mjModel * m, mjtObj type, const std::string & suffix)
  {
    int found = -1;
    int count = type == mjOBJ_JOINT ? m->njnt : m->ngeom;
    for(int i = 0; i < count; ++i)
    {
      const char * raw = mj_id2name(m, type, i);
      if(!raw) continue;
      std::string name(raw);
      if(name == suffix || (name.size() > suffix.size() &&
          name.compare(name.size()-suffix.size(), suffix.size(), suffix) == 0 &&
          name[name.size()-suffix.size()-1] == '_'))
      {
        if(found >= 0) throw std::runtime_error("Ambiguous SMPL-H model names");
        found = i;
      }
    }
    if(found < 0) throw std::runtime_error("Missing SMPL-H model element: " + suffix);
    return found;
  }
public:
  SmplhBridge() = default;
  SmplhBridge(const SmplhBridge &) = delete;
  ~SmplhBridge() { if(fd_ >= 0) ::close(fd_); }
  bool active() const { return fd_ >= 0; }
  void update(mjModel * m, mjData * d)
  {
    if(!checked_)
    {
      checked_ = true;
      const char * socketPath = std::getenv("SMPLH_SOCKET");
      if(!socketPath) return;
      const char * scenePath = std::getenv("SMPLH_SCENE");
      if(!scenePath) throw std::runtime_error("SMPLH_SCENE must name a writable local .mjb path");
      sockaddr_un addr{}; addr.sun_family = AF_UNIX;
      if(std::strlen(socketPath) >= sizeof(addr.sun_path)) throw std::runtime_error("SMPLH_SOCKET path too long");
      std::strcpy(addr.sun_path, socketPath);
      fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
      if(fd_ < 0 || ::connect(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)))
        throw std::runtime_error("Cannot connect to SMPL-H worker");
      timeval timeout{30,0};
      setsockopt(fd_,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
      setsockopt(fd_,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
      mj_saveModel(m, scenePath, nullptr, 0);
      uint32_t size = static_cast<uint32_t>(std::strlen(scenePath));
      transfer(&size, sizeof(size), true);
      transfer(const_cast<char *>(scenePath), size, true);
      uint32_t counts[2]; transfer(counts, sizeof(counts), false);
      if(counts[0] != static_cast<uint32_t>(m->ngeom)) throw std::runtime_error("SMPL-H scene mismatch");
      int geom = find(m,mjOBJ_GEOM,"smplh_surface");
      mesh_ = m->geom_dataid[geom];
      if(mesh_ < 0 || counts[1] != static_cast<uint32_t>(m->mesh_vertnum[mesh_]))
        throw std::runtime_error("SMPL-H surface topology mismatch");
      // Vertices returned by the worker are world coordinates: fixed visual body, identity geom transform.
      mju_zero(m->geom_pos+3*geom,3); mju_zero(m->geom_quat+4*geom,4); m->geom_quat[4*geom]=1;
      for(int i=0;i<52;++i)
      {
        int joint=find(m,mjOBJ_JOINT,i==0 ? "smplh_root" : "smplh_joint_"+std::to_string(i));
        if(m->jnt_type[joint] != (i==0 ? mjJNT_FREE : mjJNT_BALL))
          throw std::runtime_error("Unexpected SMPL-H joint type");
        joints_.push_back(joint);
      }
      response_.resize(211+6*counts[1]);
    }
    if(fd_ < 0) return;
    transfer(&d->time,sizeof(double),true);
    std::vector<double> transforms(12*m->ngeom);
    for(int i=0;i<m->ngeom;++i)
    {
      std::copy(d->geom_xpos+3*i,d->geom_xpos+3*i+3,transforms.data()+12*i);
      std::copy(d->geom_xmat+9*i,d->geom_xmat+9*i+9,transforms.data()+12*i+3);
    }
    transfer(transforms.data(),transforms.size()*sizeof(double),true);
    uint32_t nextFrame;
    transfer(&nextFrame,sizeof(nextFrame),false);
    if(nextFrame == frame_) return;
    transfer(response_.data(),response_.size()*sizeof(float),false);
    size_t cursor=0;
    for(size_t i=0;i<joints_.size();++i)
    {
      int j=joints_[i], nq=i==0 ? 7 : 4, nv=i==0 ? 6 : 3;
      for(int k=0;k<nq;++k) d->qpos[m->jnt_qposadr[j]+k]=static_cast<double>(response_[cursor+k]);
      mju_zero(d->qvel+m->jnt_dofadr[j],nv);
      cursor+=nq;
    }
    int n=m->mesh_vertnum[mesh_], va=m->mesh_vertadr[mesh_], na=m->mesh_normaladr[mesh_];
    if(m->mesh_normalnum[mesh_] != n) throw std::runtime_error("Surface needs one normal per vertex");
    for(int i=0;i<3*n;++i)
    {
      m->mesh_vert[3*va+i]=response_[211+i];
      m->mesh_normal[3*na+i]=response_[211+3*n+i];
    }
    mj_forward(m,d);
    frame_=nextFrame;
    mesh_dirty_=true;
  }
  void upload(const mjModel * m, mjrContext * context)
  {
    if(fd_ >= 0 && mesh_dirty_)
    {
      mjr_uploadMesh(m,context,mesh_);
      mesh_dirty_=false;
    }
  }
};
