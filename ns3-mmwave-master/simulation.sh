 #!/bin/bash
 
 
 for dist in $(seq 10 10 130)
 do
  for i in {0..99};
   do
     NS_GLOBAL_VALUE="RngRun=$i" ./waf --run "scratch/main-script --dist=$dist"
   done  
  done
