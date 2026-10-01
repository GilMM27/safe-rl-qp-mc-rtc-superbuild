#!/usr/bin/env python3

import argparse
import pathlib
import xml.etree.ElementTree as ET


def expand_children(parent, source_file, source_root, install_root):
    for child in list(parent):
        if child.tag == "include":
            include_file = (source_file.parent / child.attrib["file"]).resolve()
            included_root = ET.parse(include_file).getroot()
            index = list(parent).index(child)
            parent.remove(child)
            expanded = list(included_root)
            matching_container = next(
                (candidate for candidate in expanded if candidate.tag == parent.tag), None
            ) if parent.tag == "mujoco" else None
            if matching_container is not None:
                expanded = list(matching_container)
                for included_child in expanded:
                    expand_tree(included_child, include_file, source_root, install_root)
                for offset, included_child in enumerate(expanded):
                    parent.insert(index + offset, included_child)
            elif parent.tag == "mujoco":
                for included_container in expanded:
                    target = parent.find(included_container.tag)
                    if target is None:
                        expand_tree(included_container, include_file, source_root, install_root)
                        parent.insert(index, included_container)
                        index += 1
                    else:
                        children = list(included_container)
                        for included_child in children:
                            expand_tree(included_child, include_file, source_root, install_root)
                            target.append(included_child)
            else:
                for included_child in expanded:
                    expand_tree(included_child, include_file, source_root, install_root)
                for offset, included_child in enumerate(expanded):
                    parent.insert(index + offset, included_child)
        else:
            expand_tree(child, source_file, source_root, install_root)


def expand_tree(node, source_file, source_root, install_root):
    if node.tag in {"mesh", "texture", "skin"} and "file" in node.attrib:
        source_asset = (source_file.parent / node.attrib["file"]).resolve()
        relative_asset = source_asset.relative_to(source_root)
        node.attrib["file"] = str(install_root / relative_asset)
    expand_children(node, source_file, source_root, install_root)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--source-root", type=pathlib.Path, required=True)
    parser.add_argument("--install-root", type=pathlib.Path, required=True)
    args = parser.parse_args()

    root = ET.parse(args.input).getroot()
    expand_tree(root, args.input.resolve(), args.source_root.resolve(), args.install_root.resolve())
    default_joint = root.find("./default/joint")
    if default_joint is not None:
        # mc_mujoco merges object defaults with robot defaults. Unbounded robot
        # joints must not inherit the human model's limited=true default.
        default_joint.set("limited", "false")
    # The object is driven by qpos playback rather than MuJoCo actuators.
    # Removing cross-referenced dynamic sections also avoids mc_mujoco's
    # repeated prefixing of tendons when it merges later robot models.
    for section in ("contact", "equality", "tendon", "actuator", "sensor", "keyframe"):
        node = root.find(section)
        if node is not None:
            root.remove(node)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(args.output, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
