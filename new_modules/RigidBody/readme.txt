To replace the DUECA built-in RigidBody class and associated 
integrate_rungekutta functions which use the antiquated MTL,
this version uses the easier to use Eigen matrix library.

The DUECA documentation is still mostly valid for the big picture,
but you can generate documentation for this code with "make doc"
and look under html/. 

Use the EigenRigidBody class to derive from, make an
EigenRungeKuttaWorkspace and call integrate_rungekutta_eigen to
integrate your model.

The PulsedBody example still uses the MTL and has been removed from 
the code. You can still study it in the DUECA docs for guidance.
