### Tips and Tricks for MPCC 

#### Parameters

- q_c: the highest it can go without modifying the "optimality" of the trajectory (apexing)
- q_l: quite high
- gamma (coefficient of track advancement): it's the most sensitive 
- nu_soft: high very good for forces?


#### General Notes

- be ABSOLUTELY sure your measurement is within the constraint set. Example: you set a maximum speed and then the car can get over this speed for certain moment; feeding this to the MPC will bring it to fail and you will have the hardest time at understanding why!
