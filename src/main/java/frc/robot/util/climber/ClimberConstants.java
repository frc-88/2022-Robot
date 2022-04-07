package frc.robot.util.climber;

public class ClimberConstants {

    
    public static final ClimberState S_STOWED = new ClimberState(29.5, 27.5, 29.5, 27.5);

    public static final ClimberStateMachine M_STOW = new ClimberStateMachine(S_STOWED);

    

    public static final ClimberState S_LOW_MID_PREP = new ClimberState(0, 29, 0, 29);
    
    public static final ClimberStateMachine M_PREP_LOW_MID = new ClimberStateMachine(S_LOW_MID_PREP);

    
    public static final ClimberState S_HIGH_TRAVERSAL_FORWARDS_PREP = new ClimberState(-3, 29, 10, 30);

    public static final ClimberStateMachine M_PREP_HIGH_TRAVERSAL_FORWARDS = new ClimberStateMachine(S_HIGH_TRAVERSAL_FORWARDS_PREP);


    public static final ClimberState S_HIGH_TRAVERSAL_REVERSE_PREP = new ClimberState(-3, 29, -10, 30);

    public static final ClimberStateMachine M_PREP_HIGH_TRAVERSAL_REVERSE = new ClimberStateMachine(S_HIGH_TRAVERSAL_REVERSE_PREP);



    public static final ClimberState S_LOW_RAISED = new ClimberState(0, 33, 0, 33);

    public static final ClimberStateMachine M_RAISE_LOW = new ClimberStateMachine(S_LOW_RAISED);


    public static final ClimberState S_MID_RAISED = new ClimberState(0, 50, 0, 50);

    public static final ClimberStateMachine M_RAISE_MID = new ClimberStateMachine(S_MID_RAISED);


    public static final ClimberState S_HIGH_TRAVERSAL_FORWARDS_RAISED = new ClimberState(-3, 49, 10, 37);

    public static final ClimberStateMachine M_RAISE_HIGH_TRAVERSAL_FORWARDS = new ClimberStateMachine(S_HIGH_TRAVERSAL_FORWARDS_RAISED);


    public static final ClimberState S_HIGH_TRAVERSAL_REVERSE_RAISED = new ClimberState(10, 51, -28, 52);

    public static final ClimberStateMachine M_RAISE_HIGH_TRAVERSAL_REVERSE = new ClimberStateMachine(S_HIGH_TRAVERSAL_REVERSE_RAISED);



    public static final ClimberState S_LOW_CLIMBED = new ClimberState(0, 27, 0, 27);

    public static final ClimberStateMachine M_CLIMB_LOW = new ClimberStateMachine(S_LOW_CLIMBED);


    public static final ClimberState S_MID_CLIMBED = new ClimberState(0, 27, 0, 27);

    public static final ClimberStateMachine M_CLIMB_MID = new ClimberStateMachine(S_MID_CLIMBED);


    public static final ClimberState S_HIGH_FORWARDS_FIRST_LIFT = new ClimberState(-3, 29.5, 10, 55, ClimberState.DEFAULT_PIVOT_TOLERANCE, 6, false);
    public static final ClimberState S_HIGH_FORWARDS_HIT_HIGH_BAR = new ClimberState(-3, 29.5, 28, 55);
    public static final ClimberState S_HIGH_FORWARDS_LIFT_ONTO_HIGH_BAR = new ClimberState(-3, 29.5, 28, 50);
    public static final ClimberState S_HIGH_FORWARDS_TRANSITION_TO_HIGH_BAR = new ClimberState(-35, 34, -3, 43, ClimberState.DEFAULT_PIVOT_TOLERANCE, ClimberState.DEFAULT_TELESCOPE_TOLERANCE, true);
    public static final ClimberState S_HIGH_FORWARDS_RAISE_ABOVE_MID_BAR = new ClimberState(-35, 43, -3, 43);
    public static final ClimberState S_HIGH_FORWARDS_PIVOT_BEHIND_MID_BAR = new ClimberState(-47.9, 43, 5, 43);
    public static final ClimberState S_HIGH_FORWARDS_LOWER_BELOW_MID_BAR = new ClimberState(-47.9, 29, 5, 43, ClimberState.DEFAULT_PIVOT_TOLERANCE, 4, false);
    public static final ClimberState S_HIGH_FORWARDS_PIVOT_UNDER_MID_BAR = new ClimberState(-15, 29, 5, 43, 7, ClimberState.DEFAULT_TELESCOPE_TOLERANCE, false);
    public static final ClimberState S_HIGH_FORWARDS_RAISE_TO_HIGH_BAR = new ClimberState(-15, 52, -3, 43, ClimberState.DEFAULT_PIVOT_TOLERANCE, 8, false);
    public static final ClimberState S_HIGH_FORWARDS_PIVOT_ONTO_HIGH_BAR = new ClimberState(-3, 52, -3, 43, 6, ClimberState.DEFAULT_TELESCOPE_TOLERANCE, false);
    public static final ClimberState S_HIGH_FORWARDS_MATCH_HEIGHTS = new ClimberState(-3, 43, -3, 43, ClimberState.DEFAULT_PIVOT_TOLERANCE, 10, false);
    public static final ClimberState S_HIGH_FORWARDS_LIFT_FULLY = new ClimberState(-3, 28.5, -3, 28.5);

    public static final ClimberStateMachine M_CLIMB_HIGH_FORWARDS_TRANSFERABLE = new ClimberStateMachine(S_HIGH_FORWARDS_FIRST_LIFT)
                                                            .addState(S_HIGH_FORWARDS_HIT_HIGH_BAR)
                                                            .addState(S_HIGH_FORWARDS_LIFT_ONTO_HIGH_BAR)
                                                            .addState(S_HIGH_FORWARDS_TRANSITION_TO_HIGH_BAR)
                                                            .addState(S_HIGH_FORWARDS_RAISE_ABOVE_MID_BAR)
                                                            .addState(S_HIGH_FORWARDS_PIVOT_BEHIND_MID_BAR)
                                                            .addState(S_HIGH_FORWARDS_LOWER_BELOW_MID_BAR);
    public static final ClimberStateMachine M_CLIMB_HIGH_FORWARDS = M_CLIMB_HIGH_FORWARDS_TRANSFERABLE
                                                            .addState(S_HIGH_FORWARDS_PIVOT_UNDER_MID_BAR)
                                                            .addState(S_HIGH_FORWARDS_RAISE_TO_HIGH_BAR)
                                                            .addState(S_HIGH_FORWARDS_PIVOT_ONTO_HIGH_BAR)
                                                            .addState(S_HIGH_FORWARDS_MATCH_HEIGHTS)
                                                            .addState(S_HIGH_FORWARDS_LIFT_FULLY);


    public static final ClimberState S_TRAVERSAL_FORWARDS_PIVOT_UNDER_MID_BAR = new ClimberState(15, 35, 5, 43, 7, ClimberState.DEFAULT_TELESCOPE_TOLERANCE, false);
    public static final ClimberState S_TRAVERSAL_FORWARDS_LIFT_FULLY_ONTO_HIGH_BAR = new ClimberState(15, 55, -3, 29.5, ClimberState.DEFAULT_PIVOT_TOLERANCE, 6, false);
    public static final ClimberState S_TRAVERSAL_FORWARDS_HIT_TRAVERSAL_BAR = new ClimberState(30, 55, -10, 29.5);
    public static final ClimberState S_TRAVERSAL_FORWARDS_LIFT_ONTO_TRAVERSAL_BAR = new ClimberState(30, 50, -10, 29.5);
    public static final ClimberState S_TRAVERSAL_FORWARDS_TRANSITION_TO_TRAVERSAL_BAR = new ClimberState(-3, 43, -35, 34, ClimberState.DEFAULT_PIVOT_TOLERANCE, ClimberState.DEFAULT_TELESCOPE_TOLERANCE, true);
    public static final ClimberState S_TRAVERSAL_FORWARDS_RAISE_ABOVE_HIGH_BAR = new ClimberState(-3, 43, -35, 46);
    public static final ClimberState S_TRAVERSAL_FORWARDS_PIVOT_BEHIND_HIGH_BAR = new ClimberState(5, 43, -47.9, 46);
    public static final ClimberState S_TRAVERSAL_FORWARDS_LOWER_BELOW_HIGH_BAR = new ClimberState(5, 43, -47.9, 29, ClimberState.DEFAULT_PIVOT_TOLERANCE, 4, false);
    public static final ClimberState S_TRAVERSAL_FORWARDS_PIVOT_UNDER_HIGH_BAR = new ClimberState(5, 43, -15, 29, 7, ClimberState.DEFAULT_TELESCOPE_TOLERANCE, false);
    public static final ClimberState S_TRAVERSAL_FORWARDS_RAISE_TO_TRAVERSAL_BAR = new ClimberState(-3, 43, -15, 52, ClimberState.DEFAULT_PIVOT_TOLERANCE, 8, false);
    public static final ClimberState S_TRAVERSAL_FORWARDS_PIVOT_ONTO_TRAVERSAL_BAR = new ClimberState(-3, 43, 0, 52, 6, ClimberState.DEFAULT_TELESCOPE_TOLERANCE, false);
    public static final ClimberState S_TRAVERSAL_FORWARDS_MATCH_HEIGHTS = new ClimberState(0, 43, 0, 43, ClimberState.DEFAULT_PIVOT_TOLERANCE, 10, false);
    public static final ClimberState S_TRAVERSAL_FORWARDS_LIFT_FULLY = new ClimberState(0, 28.5, 0, 28.5);

    public static final ClimberStateMachine M_CLIMB_TRAVERSAL_FORWARDS = M_CLIMB_HIGH_FORWARDS_TRANSFERABLE
                                                            .addState(S_TRAVERSAL_FORWARDS_PIVOT_UNDER_MID_BAR)
                                                            .addState(S_TRAVERSAL_FORWARDS_LIFT_FULLY_ONTO_HIGH_BAR)
                                                            .addState(S_TRAVERSAL_FORWARDS_HIT_TRAVERSAL_BAR)
                                                            .addState(S_TRAVERSAL_FORWARDS_LIFT_ONTO_TRAVERSAL_BAR)
                                                            .addState(S_TRAVERSAL_FORWARDS_TRANSITION_TO_TRAVERSAL_BAR)
                                                            .addState(S_TRAVERSAL_FORWARDS_RAISE_ABOVE_HIGH_BAR)
                                                            .addState(S_TRAVERSAL_FORWARDS_PIVOT_BEHIND_HIGH_BAR)
                                                            .addState(S_TRAVERSAL_FORWARDS_LOWER_BELOW_HIGH_BAR)
                                                            .addState(S_TRAVERSAL_FORWARDS_PIVOT_UNDER_HIGH_BAR)
                                                            .addState(S_TRAVERSAL_FORWARDS_RAISE_TO_TRAVERSAL_BAR)
                                                            .addState(S_TRAVERSAL_FORWARDS_PIVOT_ONTO_TRAVERSAL_BAR)
                                                            .addState(S_TRAVERSAL_FORWARDS_MATCH_HEIGHTS)
                                                            .addState(S_TRAVERSAL_FORWARDS_LIFT_FULLY);


    public static final ClimberState S_HIGH_REVERSE_FIRST_LIFT = new ClimberState(10, 27, -28, 52);
    public static final ClimberState S_HIGH_REVERSE_HIT_HIGH_BAR = new ClimberState(10, 27, -20, 52);
    public static final ClimberState S_HIGH_REVERSE_LIFT_ONTO_HIGH_BAR = new ClimberState(-15, 35, 0, 38);
    public static final ClimberState S_HIGH_REVERSE_LIFT_ABOVE_HIGH_BAR = new ClimberState(-15, 44, 0, 38);
    public static final ClimberState S_HIGH_REVERSE_PIVOT_ONTO_HIGH_BAR = new ClimberState(0, 44, -3, 38);
    public static final ClimberState S_HIGH_REVERSE_LIFT_FULLY = new ClimberState(0, 27, -3, 27);

    public static final ClimberStateMachine M_CLIMB_HIGH_REVERSE_TRANSFERABLE = new ClimberStateMachine(S_HIGH_REVERSE_FIRST_LIFT)
                                                            .addState(S_HIGH_REVERSE_HIT_HIGH_BAR);
    public static final ClimberStateMachine M_CLIMB_HIGH_REVERSE = M_CLIMB_HIGH_REVERSE_TRANSFERABLE
                                                            .addState(S_HIGH_REVERSE_LIFT_ONTO_HIGH_BAR)
                                                            .addState(S_HIGH_REVERSE_LIFT_ABOVE_HIGH_BAR)
                                                            .addState(S_HIGH_REVERSE_PIVOT_ONTO_HIGH_BAR)
                                                            .addState(S_HIGH_REVERSE_LIFT_FULLY);

    public static final ClimberState S_TRAVERSAL_REVERSE_LIFT_ONTO_HIGH_BAR = new ClimberState(10, 27, -20, 48);
    public static final ClimberState S_TRAVERSAL_REVERSE_TRANSITION_TO_HIGH_BAR = new ClimberState(31, 31, 7.9, 44, 15, ClimberState.DEFAULT_TELESCOPE_TOLERANCE, true);
    public static final ClimberState S_TRAVERSAL_REVERSE_RAISE_ABOVE_MID_BAR = new ClimberState(31, 37, 7.9, 44, 2, ClimberState.DEFAULT_TELESCOPE_TOLERANCE, false);
    public static final ClimberState S_TRAVERSAL_REVERSE_PIVOT_UNDER_HIGH_BAR = new ClimberState(-28, 41, 10, 44, 34, 100, false);
    public static final ClimberState S_TRAVERSAL_REVERSE_PIVOT_UNDER_TRAVERSAL_BAR = new ClimberState(-28, 44, 10, 27, ClimberState.DEFAULT_PIVOT_TOLERANCE, 100, false);
    public static final ClimberState S_TRAVERSAL_REVERSE_RAISE_ABOVE_TRAVERSAL_BAR = new ClimberState(-28, 52, 10, 27);
    public static final ClimberState S_TRAVERSAL_REVERSE_HIT_TRAVERSAL_BAR = new ClimberState(-20, 54, 10, 27);
    public static final ClimberState S_TRAVERSAL_REVERSE_LIFT_ONTO_TRAVERSAL_BAR = new ClimberState(0, 38, 10, 35);
    public static final ClimberState S_TRAVERSAL_REVERSE_LIFT_ABOVE_TRAVERSAL_BAR = new ClimberState(0, 38, -15, 44);
    public static final ClimberState S_TRAVERSAL_REVERSE_PIVOT_ONTO_TRAVERSAL_BAR = new ClimberState(0, 38, -3, 44);
    public static final ClimberState S_TRAVERSAL_REVERSE_LIFT_FULLY = new ClimberState(0, 27, -3, 27);
    

    public static final ClimberStateMachine M_CLIMB_TRAVERSAL_REVERSE = M_CLIMB_HIGH_REVERSE_TRANSFERABLE
                                                            .addState(S_TRAVERSAL_REVERSE_TRANSITION_TO_HIGH_BAR)
                                                            .addState(new ClimberState(0.3))
                                                            .addState(S_TRAVERSAL_REVERSE_RAISE_ABOVE_MID_BAR)
                                                            .addState(S_TRAVERSAL_REVERSE_PIVOT_UNDER_HIGH_BAR)
                                                            .addState(S_TRAVERSAL_REVERSE_PIVOT_UNDER_TRAVERSAL_BAR)
                                                            .addState(S_TRAVERSAL_REVERSE_RAISE_ABOVE_TRAVERSAL_BAR)
                                                            .addState(S_TRAVERSAL_REVERSE_HIT_TRAVERSAL_BAR)
                                                            .addState(new ClimberState(0.1))
                                                            .addState(S_TRAVERSAL_REVERSE_LIFT_ONTO_TRAVERSAL_BAR)
                                                            .addState(S_TRAVERSAL_REVERSE_LIFT_ABOVE_TRAVERSAL_BAR)
                                                            .addState(S_TRAVERSAL_REVERSE_PIVOT_ONTO_TRAVERSAL_BAR)
                                                            .addState(new ClimberState(0.5))
                                                            .addState(S_TRAVERSAL_REVERSE_LIFT_FULLY);
}
