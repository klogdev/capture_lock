#include "base/database.h"
#include "base/database_cache.h"
#include "sfm/incremental_mapper.h"
#include "sfm/incremental_triangulator.h"

int main(int argc, char** argv){
    if (argc < 2)
    {
        std::cout << "Standard Input" << std::endl;
        std::cout << " " << argv[0] << "need a path to specify the database" 
                  << std::endl;
    }
    std::string file_path = argv[1];
    colmap::Database db = colmap::Database(file_path);
    colmap::DatabaseCache db_cache = colmap::DatabaseCache();
}