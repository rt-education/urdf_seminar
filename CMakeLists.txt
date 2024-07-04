cmake_minimum_required(VERSION 3.5)
project(urdf_seminar)

install(DIRECTORY
  launch
  urdf
  meshes
  worlds
  config
  DESTINATION share/${PROJECT_NAME}/
)
