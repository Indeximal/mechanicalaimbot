# Pseudocode
exit()



load_graph()

with session:
    init_graph()
    with mms:
        while True:
            handle_pygame()

            if calibrated:
                get_frame()
                run_inference()
                send_to_hardware()
            else:
                calibrate_hareware()

