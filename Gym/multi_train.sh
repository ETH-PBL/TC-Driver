#!/cluster/apps/sfos/bin/bash

# run everything n-times with provided via command-line
for (( i=0; i<$1; i++ ))
do
  # angles
  for ang in 60 90
  do 
    for mvel in 10
    do
      for epl in 10000
      do
        # bsub -W 9:00 -R "rusage[mem=3000]" python ~/edoardo-ghignone/pbl_f110_gym/train.py --ep_len 500 --ang_deg $ang --max_vel 10 --no-record_vid
        # bsub -W 9:00 -R "rusage[mem=3000]" python ~/edoardo-ghignone/pbl_f110_gym/train.py --ep_len 10000 --ang_deg $ang --max_vel 10 --no-use_traj --no-record_vid
        bsub -W 9:00 -R "rusage[mem=3000]" python ~/edoardo-ghignone/pbl_f110_gym/train.py --ep_len $epl --ang_deg $ang --max_vel $mvel --no-record_vid --no-p_noise
        bsub -W 9:00 -R "rusage[mem=3000]" python ~/edoardo-ghignone/pbl_f110_gym/train.py --ep_len $epl --ang_deg $ang --max_vel $mvel --no-record_vid --p_noise
      done
    done
  done
done
