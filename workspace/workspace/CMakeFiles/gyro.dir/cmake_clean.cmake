file(REMOVE_RECURSE
  "libgyro.a"
  "libgyro.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/gyro.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
