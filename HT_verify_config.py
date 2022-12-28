def config_verification(ipadd, drone_address="radio://0/80/2M/E7E7E7E7xx", pos_sys='Qualisys', Te=0.1):
    # pos_sys = "Qualisys"
    # Te = 0.1
    list_config = {
        "Position system:": [pos_sys, ''],
        "Controller sampling time:": [Te, 'second'],
        "Drone address": [drone_address, ""],
    }
    if pos_sys == "Qualisys":
        list_config["IP address of Qualisys"] = [ipadd, ""]
    print("\n\nPlease check CAREFULLY your configuration below")
    col = "{:<30} {:<35} {:<10}"
    title_table = col.format("Configuration", "Value", "Unit")
    print("-" * len(title_table))
    print(title_table)
    print("-" * len(title_table))
    for k, v in list_config.items():
        val, unit = v
        print(col.format(k, val, unit))
    print("-" * len(title_table))
    print('Please check also the followings: Calibration for QS, wifi connection and battery')
    user_s = input("Do you want to continue [Y/n]?")
    if user_s == "Y":
        print('Configuration accepted, carrying out the experiment . . . . . . . . .')
    else:
        raise ValueError('Experiment terminated by user, closing controller . . . ')
