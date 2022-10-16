function initial_points()
    --x_reached = 0
    --y_reached = 0
    initial_position = sim.getObjectPose(targetObj,ref)
    return initial_position
end

function sysCall_init() 
    particlesAreVisible=true
    simulateParticles=true
    fakeShadow=true
    x_reached = 0
    y_reached = 0
    counter = 0
    particleCountPerSecond=430
    particleSize=0.005
    particleDensity=8500
    particleScatteringAngle=30
    particleLifeTime=0.5
    maxParticleCount=50
    -- Detatch the manipulation sphere:
    targetObj=sim.getObject('./target')
    sim.setObjectParent(targetObj,-1,true)
    ref = sim.getObject('/Dummy')
    -- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example
    d=sim.getObject('./base')
    cube1 = sim.getObject('/cube1')
    cube2 = sim.getObject('/cube2')
    cube3 = sim.getObject('/cube3')
    cube4 = sim.getObject('/cube4')
    cube5 = sim.getObject('/cube5')
    cube6 = sim.getObject('/cube6')
    prox_sensor = sim.getObject('/Proximity_sensor')
    cube1_loc = sim.getObjectPosition(cube1,ref)
    cube2_loc = sim.getObjectPosition(cube2,ref)
    cube3_loc = sim.getObjectPosition(cube3,ref)
    cube4_loc = sim.getObjectPosition(cube4,ref)
    cube5_loc = sim.getObjectPosition(cube5,ref)
    cube6_loc = sim.getObjectPosition(cube6,ref)
    propellerHandles={}
    jointHandles={}
    particleObjects={-1,-1,-1,-1}
    local ttype=sim.particle_roughspheres+sim.particle_cyclic+sim.particle_respondable1to4+sim.particle_respondable5to8+sim.particle_ignoresgravity
    if not particlesAreVisible then
        ttype=ttype+sim.particle_invisible
    end
    for i=1,4,1 do
        propellerHandles[i]=sim.getObject('./propeller['..(i-1)..']/respondable')
        jointHandles[i]=sim.getObject('./propeller['..(i-1)..']/joint')
        if simulateParticles then
            particleObjects[i]=sim.addParticleObject(ttype,particleSize,particleDensity,{2,1,0.2,3,0.4},particleLifeTime,maxParticleCount,{0.3,0.7,1})
        end
    end
    heli=sim.getObject('.')

    pParam=2
    iParam=0
    dParam=0
    vParam=-2

    cumul=0
    lastE=0
    pAlphaE=0
    pBetaE=0
    psp2=0
    psp1=0

    prevEuler=0


    if (fakeShadow) then
        shadowCont=sim.addDrawingObject(sim.drawing_discpoints+sim.drawing_cyclic+sim.drawing_25percenttransparency+sim.drawing_50percenttransparency+sim.drawing_itemsizes,0.2,0,-1,1)
    end
end

function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    for i=1,#particleObjects,1 do
        sim.removeParticleObject(particleObjects[i])
    end
end 

function sysCall_sensing()
    result,distance,co_ordimates,handle,vector = sim.handleProximitySensor(prox_sensor)
    if(result == 1)
    then
        print(co_ordimates)
    end
end

function sysCall_actuation() 
    pos=sim.getObjectPosition(d,-1)
    if (fakeShadow) then
        itemData={pos[1],pos[2],0.002,0,0,1,0.2}
        sim.addDrawingObjectItem(shadowCont,itemData)
    end
    
    -- Vertical control:
    targetPos=sim.getObjectPosition(targetObj,-1)
    pos=sim.getObjectPosition(d,-1)
    l=sim.getVelocity(heli)
    e=(targetPos[3]-pos[3])
    cumul=cumul+e
    pv=pParam*e
    thrust=5.45+pv+iParam*cumul+dParam*(e-lastE)+l[3]*vParam
    lastE=e
    
    -- Horizontal control: 
    sp=sim.getObjectPosition(targetObj,d)
    m=sim.getObjectMatrix(d,-1)
    vx={1,0,0}
    vx=sim.multiplyVector(m,vx)
    vy={0,1,0}
    vy=sim.multiplyVector(m,vy)
    alphaE=(vy[3]-m[12])
    alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE)
    betaE=(vx[3]-m[12])
    betaCorr=-0.25*betaE-2.1*(betaE-pBetaE)
    pAlphaE=alphaE
    pBetaE=betaE
    alphaCorr=alphaCorr+sp[2]*0.005+1*(sp[2]-psp2)
    betaCorr=betaCorr-sp[1]*0.005-1*(sp[1]-psp1)
    psp2=sp[2]
    psp1=sp[1]
    
    -- Rotational control:
    euler=sim.getObjectOrientation(d,targetObj)
    rotCorr=euler[3]*0.1+2*(euler[3]-prevEuler)
    prevEuler=euler[3]
    
    -- Decide of the motor velocities:
    handlePropeller(1,thrust*(1-alphaCorr+betaCorr+rotCorr))
    handlePropeller(2,thrust*(1-alphaCorr-betaCorr-rotCorr))
    handlePropeller(3,thrust*(1+alphaCorr-betaCorr+rotCorr))
    handlePropeller(4,thrust*(1+alphaCorr+betaCorr-rotCorr))
    
    if((x_reached == 1) and (y_reached == 1))
    then
        --print("I have reached this point")
        counter = counter + 1
    end
    
if(counter == 0)
    then
        target_position = cube1_loc
        initial_position = initial_points()
        x_reached = 0
        y_reached = 0
    end
        
    if(counter == 1)
    then
        target_position = cube2_loc
        initial_position = initial_points()
        x_reached = 0
        y_reached = 0
    end
    
    if(counter == 2)
    then
        target_position = cube3_loc
        initial_position = initial_points()
        x_reached = 0
        y_reached = 0
    end
        
    if(counter == 3)
    then
        target_position = cube4_loc
        initial_position = initial_points()
        x_reached = 0
        y_reached = 0
    end
        
    if(counter == 4)
    then
        target_position = cube5_loc
        initial_position = initial_points()
        x_reached = 0
        y_reached = 0
    end
        
    if(counter == 5)
    then
        target_position = cube6_loc
        initial_position = initial_points()
        x_reached = 0
        y_reached = 0
    end

    
    
    if(target_position[1]>initial_position[1])
    then
    --print("Inside the first if condition")
        if(x_reached ==0)
        then
            current_position = sim.getObjectPose(targetObj,ref)
            --print(current_position[1],target_position[1])
            if(current_position[1] < (target_position[1]-0.05))
            then
                --print("moving in x direction")
                pose = {current_position[1] + sim.getSimulationTimeStep()*0.1, current_position[2], current_position[3]}
                sim.setObjectPosition(targetObj,ref,pose)
            else
                --print("stopped moving in x direction")
                pose = current_position
                sim.setObjectPosition(targetObj,ref,pose)
                x_reached = 1
            end
       end
    end
    
    if(target_position[2]>initial_position[2])
    then
    --print("Inside the first if condition for y")
        if(y_reached ==0)
        then
            current_position = sim.getObjectPose(targetObj,ref)
            print(current_position[2],target_position[2])
            if(current_position[2] < (target_position[2]-0.05))
            then
                --print("moving in y direction")
                pose = {current_position[1], current_position[2] + sim.getSimulationTimeStep()*0.1, current_position[3]}
                sim.setObjectPosition(targetObj,ref,pose)
            else
                --print("stopped moving in x direction")
                pose = current_position
                sim.setObjectPosition(targetObj,ref,pose)
                y_reached = 1
            end
       end
    end
    
    
    if(target_position[1]<initial_position[1])
    then
    --print("Inside the first if condition")
        if(x_reached ==0)
        then
            current_position = sim.getObjectPose(targetObj,ref)
            if(current_position[1] > (target_position[1]+0.05))
            then
                --print("moving in -x direction")
                pose = {current_position[1] - sim.getSimulationTimeStep()*0.1, current_position[2], current_position[3]}
                sim.setObjectPosition(targetObj,ref,pose)
            else
                --print("stopped moving in x direction")
                pose = current_position
                sim.setObjectPosition(targetObj,ref,pose)
                x_reached = 1
            end
       end
    end
    
    if(target_position[2]<initial_position[2])
    then
    --print("Inside the first if condition for y")
        if(y_reached ==0)
        then
            current_position = sim.getObjectPose(targetObj,ref)
            if(current_position[2] > (target_position[2]+0.05))
            then
                --print("moving in -y direction")
                pose = {current_position[1], current_position[2] - sim.getSimulationTimeStep()*0.1, current_position[3]}
                sim.setObjectPosition(targetObj,ref,pose)
            else
                --print("stopped moving in x direction")
                pose = current_position
                sim.setObjectPosition(targetObj,ref,pose)
                y_reached = 1
            end
       end
    end
       
    
    --print(counter)
end 


function handlePropeller(index,particleVelocity)
    propellerRespondable=propellerHandles[index]
    propellerJoint=jointHandles[index]
    propeller=sim.getObjectParent(propellerRespondable)
    particleObject=particleObjects[index]
    maxParticleDeviation=math.tan(particleScatteringAngle*0.5*math.pi/180)*particleVelocity
    notFullParticles=0

    local t=sim.getSimulationTime()
    sim.setJointPosition(propellerJoint,t*10)
    ts=sim.getSimulationTimeStep()
    
    m=sim.getObjectMatrix(propeller,-1)
    particleCnt=0
    pos={0,0,0}
    dir={0,0,1}
    
    requiredParticleCnt=particleCountPerSecond*ts+notFullParticles
    notFullParticles=requiredParticleCnt % 1
    requiredParticleCnt=math.floor(requiredParticleCnt)
    while (particleCnt<requiredParticleCnt) do
        -- we want a uniform distribution:
        x=(math.random()-0.5)*2
        y=(math.random()-0.5)*2
        if (x*x+y*y<=1) then
            if (simulateParticles) then
                pos[1]=x*0.08
                pos[2]=y*0.08
                pos[3]=-particleSize*0.6
                dir[1]=pos[1]+(math.random()-0.5)*maxParticleDeviation*2
                dir[2]=pos[2]+(math.random()-0.5)*maxParticleDeviation*2
                dir[3]=pos[3]-particleVelocity*(1+0.2*(math.random()-0.5))
                pos=sim.multiplyVector(m,pos)
                dir=sim.multiplyVector(m,dir)
                itemData={pos[1],pos[2],pos[3],dir[1],dir[2],dir[3]}
                sim.addParticleObjectItem(particleObject,itemData)
            end
            particleCnt=particleCnt+1
        end
    end
    -- Apply a reactive force onto the body:
    totalExertedForce=particleCnt*particleDensity*particleVelocity*math.pi*particleSize*particleSize*particleSize/(6*ts)
    force={0,0,totalExertedForce}
    m[4]=0
    m[8]=0
    m[12]=0
    force=sim.multiplyVector(m,force)
    local rotDir=1-math.mod(index,2)*2
    torque={0,0,rotDir*0.002*particleVelocity}
    torque=sim.multiplyVector(m,torque)
    sim.addForceAndTorque(propellerRespondable,force,torque)
end
