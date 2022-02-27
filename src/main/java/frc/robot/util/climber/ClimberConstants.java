package frc.robot.util.climber;

public class ClimberConstants {
    
    public static final ClimberState S_STOWED = new ClimberState(29.5, 27.5, 29.5, 27.5);
    public static final ClimberState S_STOWED_UP = new ClimberState(28.5, 30, 28.5, 30);
    public static final ClimberState S_UNSTOWED = new ClimberState(15, 30, 15, 30);

    public static final ClimberStateMachine M_STOW = new ClimberStateMachine(S_UNSTOWED)
                                                        .addState(S_STOWED_UP)
                                                        .addState(S_STOWED);
    public static final ClimberStateMachine M_UNSTOW = new ClimberStateMachine(S_STOWED_UP)
                                                        .addState(S_UNSTOWED);

    

    public static final ClimberState S_LOW_MID_PREP = new ClimberState(0, 29.5, 0, 29.5);
    
    public static final ClimberStateMachine M_PREP_LOW_MID = new ClimberStateMachine(S_LOW_MID_PREP);
    public static final ClimberStateMachine M_PREP_LOW_MID_FROM_STOW = M_UNSTOW.addStateMachine(M_PREP_LOW_MID);

    
    public static final ClimberState S_HIGH_TRAVERSAL_PREP = new ClimberState(0, 30, 25, 35);

    public static final ClimberStateMachine M_PREP_HIGH_TRAVERSAL = new ClimberStateMachine(S_HIGH_TRAVERSAL_PREP);
    public static final ClimberStateMachine M_PREP_HIGH_TRAVERSAL_FROM_STOW = M_UNSTOW.addStateMachine(M_PREP_HIGH_TRAVERSAL);



    public static final ClimberState S_LOW_RAISED = new ClimberState(0, 33, 0, 33);

    public static final ClimberStateMachine M_RAISE_LOW = new ClimberStateMachine(S_LOW_RAISED);
    public static final ClimberStateMachine M_RAISE_LOW_FROM_STOW = M_UNSTOW.addStateMachine(M_RAISE_LOW);


    public static final ClimberState S_MID_RAISED = new ClimberState(0, 50, 0, 50);

    public static final ClimberStateMachine M_RAISE_MID = new ClimberStateMachine(S_MID_RAISED);
    public static final ClimberStateMachine M_RAISE_MID_FROM_STOW = M_UNSTOW.addStateMachine(M_RAISE_MID);


    public static final ClimberState S_HIGH_TRAVERSAL_RAISED = new ClimberState(0, 49, 15, 44);

    public static final ClimberStateMachine M_RAISE_HIGH_TRAVERSAL = new ClimberStateMachine(S_HIGH_TRAVERSAL_RAISED);
    public static final ClimberStateMachine M_RAISE_HIGH_TRAVERSAL_FROM_STOW = M_UNSTOW.addStateMachine(M_RAISE_HIGH_TRAVERSAL);



    public static final ClimberState S_LOW_CLIMBED = new ClimberState(0, 28.5, 0, 30);

    public static final ClimberStateMachine M_CLIMB_LOW = new ClimberStateMachine(S_LOW_CLIMBED);


    public static final ClimberState S_MID_CLIMBED = new ClimberState(0, 28.5, 0, 30);

    public static final ClimberStateMachine M_CLIMB_MID = new ClimberStateMachine(S_MID_CLIMBED);


    public static final ClimberState S_HIGH_FIRST_LIFT = new ClimberState(0, 29.5, 15, 55);
    public static final ClimberState S_HIGH_HIT_HIGH_BAR = new ClimberState(0, 29.5, 28, 55);
    public static final ClimberState S_HIGH_LIFT_ONTO_HIGH_BAR = new ClimberState(0, 29.5, 28, 50);
    public static final ClimberState S_HIGH_TRANSITION_TO_HIGH_BAR = new ClimberState(-28, 34, 0, 43, ClimberState.DEFAULT_PIVOT_TOLERANCE, ClimberState.DEFAULT_TELESCOPE_TOLERANCE, true);
    public static final ClimberState S_HIGH_RAISE_ABOVE_MID_BAR = new ClimberState(-28, 43, 0, 43);
    public static final ClimberState S_HIGH_PIVOT_BEHIND_MID_BAR = new ClimberState(-45, 43, 0, 43);
    public static final ClimberState S_HIGH_LOWER_BELOW_MID_BAR = new ClimberState(-45, 35, 0, 43);
    public static final ClimberState S_HIGH_PIVOT_UNDER_MID_BAR = new ClimberState(-15, 35, 0, 43);
    public static final ClimberState S_HIGH_RAISE_TO_HIGH_BAR = new ClimberState(-15, 52, 0, 43);
    public static final ClimberState S_HIGH_PIVOT_ONTO_HIGH_BAR = new ClimberState(0, 52, 0, 43);
    public static final ClimberState S_HIGH_MATCH_HEIGHTS = new ClimberState(0, 43, 0, 43);
    public static final ClimberState S_HIGH_LIFT_FULLY = new ClimberState(0, 28.5, 0, 28.5);

    public static final ClimberStateMachine M_CLIMB_HIGH_TRANSFERABLE = new ClimberStateMachine(S_HIGH_FIRST_LIFT)
                                                            .addState(S_HIGH_HIT_HIGH_BAR)
                                                            .addState(S_HIGH_LIFT_ONTO_HIGH_BAR)
                                                            .addState(S_HIGH_TRANSITION_TO_HIGH_BAR)
                                                            .addState(S_HIGH_RAISE_ABOVE_MID_BAR)
                                                            .addState(S_HIGH_PIVOT_BEHIND_MID_BAR)
                                                            .addState(S_HIGH_LOWER_BELOW_MID_BAR);
    public static final ClimberStateMachine M_CLIMB_HIGH = M_CLIMB_HIGH_TRANSFERABLE
                                                            .addState(S_HIGH_PIVOT_UNDER_MID_BAR)
                                                            .addState(S_HIGH_RAISE_TO_HIGH_BAR)
                                                            .addState(S_HIGH_PIVOT_ONTO_HIGH_BAR)
                                                            .addState(S_HIGH_MATCH_HEIGHTS)
                                                            .addState(S_HIGH_LIFT_FULLY);


    public static final ClimberState S_TRAVERSAL_PIVOT_UNDER_MID_BAR = new ClimberState(15, 35, 0, 43);
    public static final ClimberState S_TRAVERSAL_LIFT_FULLY_ONTO_HIGH_BAR = new ClimberState(15, 55, 0, 29.5);
    public static final ClimberState S_TRAVERSAL_HIT_TRAVERSAL_BAR = new ClimberState(30, 55, -10, 29.5);
    public static final ClimberState S_TRAVERSAL_LIFT_ONTO_TRAVERSAL_BAR = new ClimberState(30, 50, -10, 29.5);
    public static final ClimberState S_TRAVERSAL_TRANSITION_TO_TRAVERSAL_BAR = new ClimberState(0, 43, -28, 34, ClimberState.DEFAULT_PIVOT_TOLERANCE, ClimberState.DEFAULT_TELESCOPE_TOLERANCE, true);
    public static final ClimberState S_TRAVERSAL_RAISE_ABOVE_HIGH_BAR = new ClimberState(0, 43, -28, 43);
    public static final ClimberState S_TRAVERSAL_PIVOT_BEHIND_HIGH_BAR = new ClimberState(0, 43, -45, 43);
    public static final ClimberState S_TRAVERSAL_LOWER_BELOW_HIGH_BAR = new ClimberState(0, 43, -45, 35);
    public static final ClimberState S_TRAVERSAL_PIVOT_UNDER_HIGH_BAR = new ClimberState(0, 43, -15, 35);
    public static final ClimberState S_TRAVERSAL_RAISE_TO_TRAVERSAL_BAR = new ClimberState(0, 43, -15, 52);
    public static final ClimberState S_TRAVERSAL_PIVOT_ONTO_TRAVERSAL_BAR = new ClimberState(0, 43, 0, 52);
    public static final ClimberState S_TRAVERSAL_MATCH_HEIGHTS = new ClimberState(0, 43, 0, 43);
    public static final ClimberState S_TRAVERSAL_LIFT_FULLY = new ClimberState(0, 28.5, 0, 28.5);

    public static final ClimberStateMachine M_CLIMB_TRAVERSAL = M_CLIMB_HIGH_TRANSFERABLE
                                                            .addState(S_TRAVERSAL_PIVOT_UNDER_MID_BAR)
                                                            .addState(S_TRAVERSAL_LIFT_FULLY_ONTO_HIGH_BAR)
                                                            .addState(S_TRAVERSAL_HIT_TRAVERSAL_BAR)
                                                            .addState(S_TRAVERSAL_LIFT_ONTO_TRAVERSAL_BAR)
                                                            .addState(S_TRAVERSAL_TRANSITION_TO_TRAVERSAL_BAR)
                                                            .addState(S_TRAVERSAL_RAISE_ABOVE_HIGH_BAR)
                                                            .addState(S_TRAVERSAL_PIVOT_BEHIND_HIGH_BAR)
                                                            .addState(S_TRAVERSAL_LOWER_BELOW_HIGH_BAR)
                                                            .addState(S_TRAVERSAL_PIVOT_UNDER_HIGH_BAR)
                                                            .addState(S_TRAVERSAL_RAISE_TO_TRAVERSAL_BAR)
                                                            .addState(S_TRAVERSAL_PIVOT_ONTO_TRAVERSAL_BAR)
                                                            .addState(S_TRAVERSAL_MATCH_HEIGHTS)
                                                            .addState(S_TRAVERSAL_LIFT_FULLY);
}
