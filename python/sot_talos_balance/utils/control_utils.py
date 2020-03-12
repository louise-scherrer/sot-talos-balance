def setComAdmittance(robot, deltaComDes, Kp_adm=None, Kp_dcm=None, Ki_dcm=None):
    comDes = list(robot.dcm_control.dcmDes.value)
    comDes[0] += deltaComDes[0]
    comDes[1] += deltaComDes[1]
    comDes = tuple(comDes)
    dcmDes = comDes
    zmpDes = comDes[:2] + (0.0, )
    ddcomDes = (0.0, 0.0, 0.0)
    if Kp_adm is not None:
        robot.com_admittance_control.Kp.value = Kp_adm
    if Kp_dcm is not None:
        robot.dcm_control.Kp.value = Kp_dcm
    if Ki_dcm is not None:
        robot.dcm_control.Ki.value = Ki_dcm
    robot.dcm_control.dcmDes.value = dcmDes
    robot.dcm_control.zmpDes.value = zmpDes
    robot.com_admittance_control.ddcomDes.value = ddcomDes
