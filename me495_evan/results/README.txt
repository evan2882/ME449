Best
	controller:PI
	Gain: P:6*identity matrix  I:2*identity matrix


Overshoot: 
	controller: PI
	Gain: P:6*identity matrix  I:8*identity matrix

NewTask:
	controller:P
	Gain: 1*identity matrix
	cube initial position: Tsci= 
		 [1, 0, 0, 1],
                 [0, 1, 0, 1],
                 [0, 0, 1, 0.025],
                 [0, 0, 0, 1]
	cube goal position : Tscf = 
		 [0, 1, 0, 1],
                 [-1, 0, 0, -1],
                 [0, 0, 1, 0.025],
                 [0, 0, 0, 1]
	     

