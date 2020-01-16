**Drive Train**
    DriveTurn(encoderLocation, gyroLocation)

    Drive(EncoderLocation)

    Turn(gyroLocation)

**Shooter**
    Shoot(speed)

**Turret**

    toPosition(position)

    positionToZero()


**Lift**
    toPosition()
    
**Hook**
    crawl(speed, direction)

    lock()

**Magazine**
    powerCells

    run(speed)
        Runs mechanism to send power cells to shooter

    getCellCount()

    addCell()

    subtractCell()

**Collect**
    run(direction, int speed)

    lockIntake()
        prevents any power cells from being collected. Should not lock purging. 

**Map**
    X coordinate
    Y coordinate
    Array of objects: [x, y, height, boundaryZone] for each object

    FindCoordinates()

    DistanceToObject()

    addObjects()
        takes input from file list of points of interest on feild and inputs them into an array

**Video**

**Ground Vision Sensor**
    boolean pastWhiteLine()

    powerCellLocation()
        returns a double of how far the cell is from being aligned with the center of the collector
    
**Target Sensor**
    innerPortPosition

    distanceFromPort()

    trajector()
        calculates the necesary speed the shooter must spin at in order to make an inner port shot

    getInnerPortPosition()
        

**Jettson Nano Vision**

**Driver Station**
    

