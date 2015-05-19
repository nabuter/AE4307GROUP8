;; -*-scheme-*-
   (make-module 'multi-stick "" sim-priority
		'set-timing sim-timing
		'check-timing 10000 20000
		
		; indicate that we will provide all customary channels
		'use-primary #t
		'use-secondary #t
		'use-primary-switches #t
		'use-secondary-switches #t

		; the joystick device to be opened 
		'set-device "/dev/input/js0"

		; each link describes the connection between a button
		; or axis of the joystick, and the output on one of
		; the channels

		; roll control channel
		'add-link
		(make-stick-value
		 'name "ux" 'read-axis 0  ; roll is axis 0 joystick
		 ; polynomial, to get ux = 0.0 - 0.2 (joystick ax 0), i.e.
		 ; a range of -0.2 to 0.2, with inverted sign (left positive)
		 'calibration-polynomial 0.0 -0.2)
		
		; pitch
		'add-link
		(make-stick-value
		 'name "uy" 'read-axis 1
		 'calibration-polynomial 0.0 -0.3)
		
		; yaw/rudder = joystick twist
		'add-link
		(make-stick-value
		 'name "uz" 'read-axis 2
		 'calibration-polynomial 0.0 -0.3)

		; collective 
		'add-link
		(make-stick-value
		 'name "uc" 'read-axis 3
		 'calibration-polynomial 0.5 0.5)

		; throttle, left
		'add-link
		(make-stick-value
		 'name "throttle_left" 'read-axis 3
		 'calibration-polynomial 0.5 -0.5)

		; re-use axis 3 for the right throttle
		'add-link
		(make-stick-value
		 'name "throttle_right" 'read-axis 3
		 'calibration-polynomial 0.5 -0.5)
		
		; flap setting with two buttons
		'add-link
		(make-stick-value
		 'name "flap_setting"
		 ; a counter with values 0, 1, 2, 3, 4, 5
		 'create-counter 5
		 ; pressing button 4 (labeled 3 on stick) pushes counter up
		 'set-counter-up
		 (make-stick-value 'read-button 2)
		 ; pressing button 3 (labeled 2 on stick) pushes counter down
		 'set-counter-down
		 (make-stick-value 'read-button 3)
		 ; can use a step calibration, to translate 0 .. 5 into
		 ; flap settings
		 'calibration-step 
		 0 0     ; count 0 is 0 deg
		 1 1     ; count 1 is 1 deg
		 2 5     ; count 2 is 5 deg
		 3 10    ; count 3 is 10 deg
		 4 25    ; count 4 is 25 deg
		 5 40    ; count 5 is 40 deg
		 )
		)
 
