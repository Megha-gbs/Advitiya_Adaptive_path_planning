function testDynamicEgoTracking()

    % ============================================================
    % DYNAMIC EGO + MULTI-OBJECT TRACKING TEST
    %
    % Purpose:
    %   Validate sensor fusion and uncertainty-aware tracking
    %   while the ego vehicle is moving.
    %
    % Ego vehicle:
    %   Moves forward and accelerates slightly.
    %
    % Objects:
    %   1. Car
    %   2. Motorcycle
    %   3. Pedestrian
    %
    % Sensors:
    %   Camera
    %   LiDAR
    %   Radar
    %
    % Pipeline:
    %
    %   World objects
    %        ↓
    %   Ego motion
    %        ↓
    %   Relative sensor measurements
    %        ↓
    %   Camera + LiDAR + Radar
    %        ↓
    %   Sensor fusion
    %        ↓
    %   Uncertainty-aware tracking
    %
    % ============================================================


    clc;

    fprintf("\n");
    fprintf("============================================================\n");
    fprintf("DYNAMIC EGO + MULTI-OBJECT TRACKING TEST\n");
    fprintf("============================================================\n");


    % ------------------------------------------------------------
    % SIMULATION SETTINGS
    % ------------------------------------------------------------

    timeStep = 0.1;

    simulationTime = 5.0;

    timeVector = 0:timeStep:simulationTime;

    numberOfSteps = length(timeVector);


    % ------------------------------------------------------------
    % EGO VEHICLE INITIAL STATE
    %
    % World frame:
    %
    % X = forward
    % Y = lateral
    % ------------------------------------------------------------

    egoPosition = [0 0];

    egoVelocity = [8 0];

    egoAcceleration = [0.5 0];

    egoHeading = 0;


    % ------------------------------------------------------------
    % OBJECT INITIAL STATES
    %
    % All values are WORLD FRAME states.
    % ------------------------------------------------------------


    % ------------------------------------------------------------
    % CAR
    % ------------------------------------------------------------

    carPosition = [45 2];

    carVelocity = [5 0];


    % ------------------------------------------------------------
    % MOTORCYCLE
    % ------------------------------------------------------------

    motorcyclePosition = [30 -3];

    motorcycleVelocity = [6 0.5];


    % ------------------------------------------------------------
    % PEDESTRIAN
    % ------------------------------------------------------------

    pedestrianPosition = [20 5];

    pedestrianVelocity = [0 -1.2];


    % ------------------------------------------------------------
    % TRACK STORAGE
    % ------------------------------------------------------------

    tracks = [];

    trackHistory = cell(0);


    % ------------------------------------------------------------
    % DISPLAY HEADER
    % ------------------------------------------------------------

    fprintf("\n");
    fprintf("Time   EgoX   EgoV   TrackID   Class");
    fprintf("        RelX       RelY       RelVx      RelVy\n");

    fprintf("------------------------------------------------------------");
    fprintf("----------------------------------------\n");


    % ============================================================
    % MAIN SIMULATION LOOP
    % ============================================================

    for k = 1:numberOfSteps

        currentTime = timeVector(k);


        % --------------------------------------------------------
        % 1. UPDATE EGO VEHICLE
        % --------------------------------------------------------

        if k > 1

            egoVelocity = ...
                egoVelocity + ...
                egoAcceleration * timeStep;

            egoPosition = ...
                egoPosition + ...
                egoVelocity * timeStep;

        end


        % --------------------------------------------------------
        % 2. UPDATE WORLD OBJECTS
        % --------------------------------------------------------

        if k > 1

            carPosition = ...
                carPosition + ...
                carVelocity * timeStep;


            motorcyclePosition = ...
                motorcyclePosition + ...
                motorcycleVelocity * timeStep;


            pedestrianPosition = ...
                pedestrianPosition + ...
                pedestrianVelocity * timeStep;

        end


        % --------------------------------------------------------
        % 3. CALCULATE RELATIVE POSITIONS
        %
        % For this test:
        %
        % egoHeading = 0
        %
        % so the world and vehicle axes are aligned.
        %
        % Later we will add changing ego heading.
        % --------------------------------------------------------

        carRelativePosition = ...
            carPosition - egoPosition;


        motorcycleRelativePosition = ...
            motorcyclePosition - egoPosition;


        pedestrianRelativePosition = ...
            pedestrianPosition - egoPosition;


        % --------------------------------------------------------
        % 4. CALCULATE RELATIVE VELOCITIES
        % --------------------------------------------------------

        carRelativeVelocity = ...
            carVelocity - egoVelocity;


        motorcycleRelativeVelocity = ...
            motorcycleVelocity - egoVelocity;


        pedestrianRelativeVelocity = ...
            pedestrianVelocity - egoVelocity;


        % --------------------------------------------------------
        % 5. CREATE SENSOR MEASUREMENTS
        % --------------------------------------------------------

        cameraCar = createDetection( ...
            currentTime, ...
            "camera", ...
            "camera_front", ...
            "car", ...
            carRelativePosition + [0.10 -0.05], ...
            carRelativeVelocity + [0.15 -0.05], ...
            0.82, ...
            "vehicle");


        lidarCar = createDetection( ...
            currentTime, ...
            "lidar", ...
            "lidar_main", ...
            "car", ...
            carRelativePosition + [-0.03 0.02], ...
            carRelativeVelocity + [0.03 0.01], ...
            0.95, ...
            "vehicle");


        radarCar = createDetection( ...
            currentTime, ...
            "radar", ...
            "radar_front", ...
            "car", ...
            carRelativePosition + [0.05 0.03], ...
            carRelativeVelocity + [-0.05 0.02], ...
            0.90, ...
            "vehicle");


        % --------------------------------------------------------
        % MOTORCYCLE SENSORS
        % --------------------------------------------------------

        cameraMotorcycle = createDetection( ...
            currentTime, ...
            "camera", ...
            "camera_front", ...
            "motorcycle", ...
            motorcycleRelativePosition + [0.12 -0.04], ...
            motorcycleRelativeVelocity + [0.10 0.04], ...
            0.78, ...
            "vehicle");


        lidarMotorcycle = createDetection( ...
            currentTime, ...
            "lidar", ...
            "lidar_main", ...
            "motorcycle", ...
            motorcycleRelativePosition + [-0.04 0.03], ...
            motorcycleRelativeVelocity + [0.04 -0.02], ...
            0.93, ...
            "vehicle");


        radarMotorcycle = createDetection( ...
            currentTime, ...
            "radar", ...
            "radar_front", ...
            "motorcycle", ...
            motorcycleRelativePosition + [0.03 0.05], ...
            motorcycleRelativeVelocity + [-0.04 0.03], ...
            0.91, ...
            "vehicle");


        % --------------------------------------------------------
        % PEDESTRIAN SENSORS
        % --------------------------------------------------------

        cameraPedestrian = createDetection( ...
            currentTime, ...
            "camera", ...
            "camera_front", ...
            "pedestrian", ...
            pedestrianRelativePosition + [0.15 -0.08], ...
            pedestrianRelativeVelocity + [0.10 0.08], ...
            0.75, ...
            "vehicle");


        lidarPedestrian = createDetection( ...
            currentTime, ...
            "lidar", ...
            "lidar_main", ...
            "pedestrian", ...
            pedestrianRelativePosition + [-0.02 0.03], ...
            pedestrianRelativeVelocity + [0.02 -0.03], ...
            0.94, ...
            "vehicle");


        radarPedestrian = createDetection( ...
            currentTime, ...
            "radar", ...
            "radar_front", ...
            "pedestrian", ...
            pedestrianRelativePosition + [0.04 0.02], ...
            pedestrianRelativeVelocity + [-0.03 0.05], ...
            0.86, ...
            "vehicle");


        % --------------------------------------------------------
        % 6. FUSE CAR
        % --------------------------------------------------------

        carDetections = [ ...
            cameraCar ...
            lidarCar ...
            radarCar];


        fusedCar = ...
            fuseDetections(carDetections);


        % --------------------------------------------------------
        % 7. FUSE MOTORCYCLE
        % --------------------------------------------------------

        motorcycleDetections = [ ...
            cameraMotorcycle ...
            lidarMotorcycle ...
            radarMotorcycle];


        fusedMotorcycle = ...
            fuseDetections(motorcycleDetections);


        % --------------------------------------------------------
        % 8. FUSE PEDESTRIAN
        % --------------------------------------------------------

        pedestrianDetections = [ ...
            cameraPedestrian ...
            lidarPedestrian ...
            radarPedestrian];


        fusedPedestrian = ...
            fuseDetections(pedestrianDetections);


        % --------------------------------------------------------
        % 9. COMBINE FUSED DETECTIONS
        % --------------------------------------------------------

        fusedDetections = [ ...
            fusedCar ...
            fusedMotorcycle ...
            fusedPedestrian];


        % --------------------------------------------------------
        % 10. UPDATE TRACKER
        % --------------------------------------------------------

        tracks = ...
            updateTracks( ...
            tracks, ...
            fusedDetections);


        % --------------------------------------------------------
        % 11. PRINT TRACK INFORMATION
        % --------------------------------------------------------

        for i = 1:length(tracks)

            fprintf( ...
                "%4.1f  %5.1f  %5.2f  %7d   %-10s", ...
                currentTime, ...
                egoPosition(1), ...
                norm(egoVelocity), ...
                tracks(i).id, ...
                tracks(i).class);


            fprintf( ...
                " [%7.2f %7.2f]", ...
                tracks(i).position(1), ...
                tracks(i).position(2));


            fprintf( ...
                " [%8.2f %8.2f]", ...
                tracks(i).velocity(1), ...
                tracks(i).velocity(2));


            fprintf("\n");


            % ----------------------------------------------------
            % Store history
            % ----------------------------------------------------

            historyEntry.time = currentTime;

            historyEntry.egoPosition = egoPosition;

            historyEntry.egoVelocity = egoVelocity;

            historyEntry.trackId = tracks(i).id;

            historyEntry.class = tracks(i).class;

            historyEntry.position = tracks(i).position;

            historyEntry.velocity = tracks(i).velocity;

            historyEntry.positionStd = ...
                tracks(i).positionStd;

            historyEntry.velocityStd = ...
                tracks(i).velocityStd;


            trackHistory{end + 1} = ...
                historyEntry;

        end


        % --------------------------------------------------------
        % 12. CHECK TRACK COUNT
        % --------------------------------------------------------

        if length(tracks) ~= 3

            fprintf("\n");

            fprintf( ...
                "WARNING: Expected 3 tracks, got %d\n", ...
                length(tracks));

        end

    end


    % ============================================================
    % FINAL VALIDATION
    % ============================================================

    fprintf("\n");
    fprintf("============================================================\n");
    fprintf("DYNAMIC TRACKING VALIDATION\n");
    fprintf("============================================================\n");


    % ------------------------------------------------------------
    % Validation 1: Track count
    % ------------------------------------------------------------

    if length(tracks) == 3

        fprintf( ...
            "PASS: Three objects tracked simultaneously.\n");

    else

        fprintf( ...
            "FAIL: Expected 3 final tracks, got %d.\n", ...
            length(tracks));

    end


    % ------------------------------------------------------------
    % Validation 2: Track hits
    % ------------------------------------------------------------

    allTracksHaveHits = true;


    for i = 1:length(tracks)

        if tracks(i).hits < 10

            allTracksHaveHits = false;

        end

    end


    if allTracksHaveHits

        fprintf( ...
            "PASS: Tracks were continuously updated.\n");

    else

        fprintf( ...
            "FAIL: One or more tracks have insufficient hits.\n");

    end


    % ------------------------------------------------------------
    % Validation 3: Position uncertainty
    % ------------------------------------------------------------

    uncertaintyValid = true;


    for i = 1:length(tracks)

        if ~isfinite(tracks(i).positionStd)

            uncertaintyValid = false;

        end

    end


    if uncertaintyValid

        fprintf( ...
            "PASS: Position uncertainty maintained.\n");

    else

        fprintf( ...
            "FAIL: Invalid position uncertainty detected.\n");

    end


    % ------------------------------------------------------------
    % Validation 4: Velocity
    % ------------------------------------------------------------

    velocityValid = true;


    for i = 1:length(tracks)

        if any(~isfinite(tracks(i).velocity))

            velocityValid = false;

        end

    end


    if velocityValid

        fprintf( ...
            "PASS: Velocity estimates maintained.\n");

    else

        fprintf( ...
            "FAIL: Invalid velocity estimate detected.\n");

    end


    % ------------------------------------------------------------
    % Final ego state
    % ------------------------------------------------------------

    fprintf("\n");
    fprintf("FINAL EGO STATE\n");
    fprintf("--------------------------------------------\n");

    fprintf( ...
        "Position : [%.3f %.3f] m\n", ...
        egoPosition(1), ...
        egoPosition(2));


    fprintf( ...
        "Velocity : [%.3f %.3f] m/s\n", ...
        egoVelocity(1), ...
        egoVelocity(2));


    fprintf( ...
        "Speed    : %.3f m/s\n", ...
        norm(egoVelocity));


    % ------------------------------------------------------------
    % Final tracks
    % ------------------------------------------------------------

    fprintf("\n");
    fprintf("FINAL TRACK STATES\n");
    fprintf("--------------------------------------------\n");


    for i = 1:length(tracks)

        fprintf( ...
            "Track %d | %-10s | Position [%7.2f %7.2f] | ", ...
            tracks(i).id, ...
            tracks(i).class, ...
            tracks(i).position(1), ...
            tracks(i).position(2));


        fprintf( ...
            "Velocity [%6.2f %6.2f] | ", ...
            tracks(i).velocity(1), ...
            tracks(i).velocity(2));


        fprintf( ...
            "Std %.3f m\n", ...
            tracks(i).positionStd);

    end


    fprintf("\n");
    fprintf("============================================================\n");
    fprintf("DYNAMIC TRACKING TEST COMPLETE\n");
    fprintf("============================================================\n");

end