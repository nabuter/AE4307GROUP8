;; -*-scheme-*-
;; this is an example dueca.mod file, for you to start out with and adapt 
;; according to your needs. Note that you need a dueca.mod file only for the 
;; node with number 0

;; NOTE: for new guile (from openSUSE 12.1 onwards), you may only once use
;; (define ...). For modifying an already defined value, used (set! ...)

;; in general, it is a good idea to clearly document your set up
;; this is an excellent place. 
;; node set-up
(define ecs-node 0)    ; dutmms1, send order 3
;(define aux-node 1)   ; dutmms3, send order 1
;(define pfd-node 2)   ; dutmms5, send order 2
;(define cl-node 3)    ; dutmms4, send order 0

;; priority set-up
; normal nodes: 0 administration
;               1 simulation, unpackers
;               2 communication
;               3 ticker

; administration priority. Run the interface and logging here
(define admin-priority (make-priority-spec 0 0))

; priority of simulation, just above adiminstration
(define sim-priority (make-priority-spec 1 0))

; nodes with a different priority scheme
; control loading node has 0, 1 and 2 as above and furthermore
;               3 stick priority
;               4 ticker priority
; priority of the stick. Higher than prio of communication
(define stick-priority (make-priority-spec 3 0)) 

; timing set-up
; timing of the stick calculations. Assuming 500 usec ticks, this gives 2000 Hz
(define stick-timing (make-time-spec 0 1))

; this is normally 20, giving 100 Hz timing
(define sim-timing (make-time-spec 0 20))

;; for now, display on 50 Hz
(define display-timing (make-time-spec 0 40))

;; log a bit more economical, 25 Hz
(define log-timing (make-time-spec 0 80))

;;; the modules needed for dueca itself
(dueca-list
  (make-entity "dueca"
	       (if (equal? 0 this-node-id)
		   (list
		    (make-module 'dusime "" admin-priority)
		    (make-module 'dueca-view "" admin-priority)
		    (make-module 'activity-view "" admin-priority)
		    (make-module 'timing-view "" admin-priority)
		    (make-module 'log-view "" admin-priority)
		    ) 
		 (list) 
		 )
	       )
  )

;;; the modules for your application(s)
(define simple-simulation
  (make-entity "ph-simple"
	       (if (equal? ecs-node this-node-id) 
		   (list
		    (make-module 'signal-generator "" sim-priority
				 'set-timing sim-timing
				 'check-timing 10000 20000
				 'Amplitude 0.5
				 'Frequency 5.0)
		     (make-module 'display "" admin-priority
				 'set-timing display-timing
				 'check-timing 10000 20000)
		    )
		 )

               ; an empty list; at least one list should be supplied
               ; for nodes that have no modules as argument
	       (list)
	       )
  )

