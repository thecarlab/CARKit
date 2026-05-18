# CARKit Vendor Note

This folder contains vendored `ndt_omp_ros2` source used by CARKit mapping.

Upstream:

- Repository: `https://github.com/rsasaki0109/ndt_omp_ros2`
- Source basis: the version previously carried in CARKit under `src/lidarslam_ros2/Thirdparty/ndt_omp_ros2`

CARKit keeps this package in the repository, without upstream `.git` metadata, because the mapping stack may need local integration changes. Keep the upstream `LICENSE` and `README.md` files in place.

Only sensor drivers are fetched by `carkit/setup_vendor_repos.sh`.
