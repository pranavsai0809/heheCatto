function sysCall_init()
    sensorHandle=sim.getObject('/Proximity_sensor')
end

function sysCall_sensing()
    handleSensor=not handleSensor
    if handleSensor then
        sim.handleProximitySensor(sensorHandle)
    end
end
