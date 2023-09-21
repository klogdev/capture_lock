file(REMOVE_RECURSE
  "libcost.a"
  "libcost.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/cost.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
