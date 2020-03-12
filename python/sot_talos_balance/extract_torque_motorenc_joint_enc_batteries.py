import subprocess

import rosbag
import rospy
import yaml

bagname = '2017-07-24-08-04-30.bag'

info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bagname], stdout=subprocess.PIPE).communicate()[0])

bag = rosbag.Bag(bagname)

ps = bag.read_messages(topics=['/power_status'])
introdata = bag.read_messages(topics=['/data'])
start_time = rospy.Time()


class message:
    """Store torque, motor encoder, joint encoder"""
    motorenc = 0
    jointenc = 0
    torquesensor = 0
    current = 0


def extract_data():

    jointnames = ('torso_1', 'torso_2', 'head_1', 'head_2', 'arm_left_1', 'arm_left_2', 'arm_left_3', 'arm_left_4',
                  'arm_left_5', 'arm_left_6', 'arm_left_7', 'arm_right_1', 'arm_right_2', 'arm_right_3', 'arm_right_4',
                  'arm_right_5', 'arm_right_6', 'arm_right_7', 'leg_left_1', 'leg_left_2', 'leg_left_3', 'leg_left_4',
                  'leg_left_5', 'leg_left_6', 'leg_left_7', 'leg_right_1', 'leg_right_2', 'leg_right_3', 'leg_right_4',
                  'leg_right_5', 'leg_right_6', 'leg_right_7')

    filesnames = {}
    files = {}
    for jn in jointnames:
        filesnames[jn] = '/BackupFiles/devel-src/data/' + jn + '_me_je.dat'
        files[jn] = open(filesnames[jn], 'w')

    start = 1
    for apw in introdata:
        messages = {}
        if start == 1:
            start_time = apw.timestamp
            start = 0
        for jn in jointnames:
            messages[jn] = message()
        for adbdata in apw.message.doubles:
            for jn in jointnames:
                if jn + '_motor_position' == adbdata.name:
                    messages[jn].motorenc = adbdata.value
                if jn + '_motor_absolute_encoder_position' == adbdata.name:
                    messages[jn].jointenc = adbdata.value
                if jn + '_motor_torque_sensor' == adbdata.name:
                    messages[jn].torquesensor = adbdata.value
                if jn + '_motor_effort' == adbdata.name:
                    messages[jn].current = adbdata.value

        for jn in jointnames:
            afile = files[jn]
            diff_secs = apw.timestamp.secs - start_time.secs
            diff_nanosecs = apw.timestamp.nsecs - start_time.nsecs
            if (diff_nanosecs < 0):
                diff_nanosecs = 1000000000 + diff_nanosecs
                diff_secs = diff_secs - 1

            afile.write(
                str(messages[jn].motorenc) + ' ' + str(messages[jn].jointenc) + ' ' + str(messages[jn].torquesensor) +
                ' ' + str(messages[jn].current) + ' ' + str(diff_secs) + '.' + str(diff_nanosecs).zfill(9) + '\n')
    for jn in jointnames:
        files[jn].close()


def extract_batteries_data():
    f = open('/BackupFiles/devel-src/data/batteries.dat', 'w')
    introdata = bag.read_messages(topics=['/data'])
    apw = introdata.next()
    start_time = apw.timestamp
    for aps in ps:
        diff_secs = aps.timestamp.secs - start_time.secs
        diff_nanosecs = aps.timestamp.nsecs - start_time.nsecs
        if (diff_nanosecs < 0):
            diff_nanosecs = 1000000000 + diff_nanosecs
            diff_secs = diff_secs - 1

        f.write(
            str(aps.message.charge) + ' ' + str(aps.message.current) + ' ' + str(aps.message.input_voltage) + ' ' +
            str(aps.message.battery_voltage) + ' ' + str(aps.message.regulator_voltage) + ' ' +
            str(aps.message.lower_body_voltage) + ' ' + str(aps.message.charger_voltage) + ' ' +
            str(aps.message.upper_body_voltage) + ' ' + str(diff_secs) + '.' + str(diff_nanosecs).zfill(9) + '\n')

    f.close()


# main
extract_data()
extract_batteries_data()
bag.close()
