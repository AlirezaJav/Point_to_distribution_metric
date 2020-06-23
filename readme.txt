
1. Dependency
   Boost is required to compile and run the program. In particular,
   component "program_options" in Boost is required to process command line
   parameters.

2. Compiling instructions
   CMakeLists.txt is provided for cmake to generate makefiles. General
   practice using cmake should be followed in order to compile the
   program. Suggested steps to compile under Debug mode are shown below.

   :$ cd /path/to/root/folder/of/the/source/code/

   :$ ls
   changes.txt  readme.txt  source  test

   :$ mkdir debug

   :$ cd debug

   :$ cmake -DCMAKE_BUILD_TYPE=Debug ../source
   You may want to specify the location of a pariticular Boost version,
   e.g., via -DBOOST_ROOT=/path/to/boost_1_63_0

   :$ make
   "pc_error_d" to be generated under ./test folder

3. Reference
   MPEG input document M40522, "Updates and Integration of Evaluation
   Metric Software for PCC"
