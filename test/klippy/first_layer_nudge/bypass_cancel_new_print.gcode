G28
TEST_ARM_NUDGE
SET_PRINT_STATS_INFO TOTAL_LAYER=100 CURRENT_LAYER=1
; Model a lower-level bypass ending the armed file, then a different file
; starting without PRINT_START: the recorded identity is stale.
SET_GCODE_VARIABLE MACRO=FIRST_LAYER_NUDGE VARIABLE=armed_filename VALUE='"cancelled.gcode"'
ASSERT_NUDGE NET=0 BASELINE=0 Z=0 ARMED=1
EXPECT_REFUSAL ARMED_AFTER=0
FIRST_LAYER_NUDGE DIRECTION=CLOSER
