function sysCall_init()
    if simROS then
        sim.addLog(sim.verbosity_msgs,"ROS interface was found.")
        startSub=simROS.subscribe('/startSimulation', 'std_msgs/Bool', 'startSimulation_callback')
        pauseSub=simROS.subscribe('/pauseSimulation', 'std_msgs/Bool', 'pauseSimulation_callback')
        stopSub=simROS.subscribe('/stopSimulation', 'std_msgs/Bool', 'stopSimulation_callback')
        enableSynModeSub=simROS.subscribe('/enableSyncMode', 'std_msgs/Bool', 'enableSyncMode_callback')
        triggerNextStepSub=simROS.subscribe('/triggerNextStep', 'std_msgs/Bool', 'triggerNextStep_callback')

        simStepDonePub=simROS.advertise('/simulationStepDone', 'std_msgs/Bool')
        simStatePub=simROS.advertise('/simulationState','std_msgs/Int32')
        simTimePub=simROS.advertise('/simulationTime','std_msgs/Float32')
        auxPub=simROS.advertise('/privateMsgAux', 'std_msgs/Bool')
        auxSub=simROS.subscribe('/privateMsgAux', 'std_msgs/Bool', 'aux_callback')
        
        rosInterfaceSynModeEnabled=false
        haltMainScript=false
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS interface was not found. Cannot run.")
    end
end
function startSimulation_callback(msg)
    sim.startSimulation()
end

function pauseSimulation_callback(msg)
    sim.pauseSimulation()
end

function stopSimulation_callback(msg)
    sim.stopSimulation()
end

function enableSyncMode_callback(msg)
    rosInterfaceSynModeEnabled=msg.data
    haltMainScript=rosInterfaceSynModeEnabled
end

function triggerNextStep_callback(msg)
    haltMainScript=false
end

function aux_callback(msg)
    simROS.publish(simStepDonePub,{data=true})
end

function publishSimState()
    local state=0 -- simulation not running
    local s=sim.getSimulationState()
    if s==sim.simulation_paused then
        state=2 -- simulation paused
    elseif s==sim.simulation_stopped then
        state=0 -- simulation stopped
    else
        state=1 -- simulation running
    end
    simROS.publish(simStatePub,{data=state})
end


function sysCall_nonSimulation()
    if simROS then
        publishSimState()
    end
end

function sysCall_beforeMainScript()
    return {doNotRunMainScript=haltMainScript}
end

function sysCall_actuation()
    if simROS then
        publishSimState()
        simROS.publish(simTimePub,{data=sim.getSimulationTime()})
    end
end

function sysCall_sensing()
    if simROS then
        simROS.publish(auxPub,{data=true})
        haltMainScript=rosInterfaceSynModeEnabled
    end
end

function sysCall_suspended()
    if simROS then
        publishSimState()
    end
end

function sysCall_afterSimulation()
    if simROS then
        publishSimState()
    end
end

function sysCall_cleanup()
    if simROS then
        simROS.shutdownSubscriber(startSub)
        simROS.shutdownSubscriber(pauseSub)
        simROS.shutdownSubscriber(stopSub)
        simROS.shutdownSubscriber(enableSynModeSub)
        simROS.shutdownSubscriber(triggerNextStepSub)
        simROS.shutdownSubscriber(auxSub)
        simROS.shutdownPublisher(auxPub)
        simROS.shutdownPublisher(simStepDonePub)
        simROS.shutdownPublisher(simStatePub)
        simROS.shutdownPublisher(simTimePub)
    end
end


