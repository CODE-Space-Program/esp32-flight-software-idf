def test_all_unity_cases(dut):
    """
    Press ENTER, enumerate & run every TEST_CASE in the unit-test-app.
    Will print each case’s name and pass/fail result.
    """
    dut.run_all_single_board_cases()
