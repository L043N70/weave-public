public class WEAVE {

    private final int[][] stationsRegion;
    private final Region region;
    private final int[][] droneRegion;
    private final int numOfDrones;
    private final int rows;
    private final int cols;
    private final IUAV dronesSpecifications;
    private final IBSS chargingStationSpecifications;
    private final int numOfObstaclesCells;
    private final ConsolePanel console;

    private ArrayList<Integer[]> startingPositions;
    private boolean[][] dronePositionMatrix;
    private int[][] assignmentMatrix;
    private ArrayList<boolean[][]> dronesRegions;
    private boolean isAbort, isSuccess;
    private ArrayList<ArrayList<Integer[]>> dronesPaths;
    private ArrayList<Vector<?>> kruskalMSTs;
    private ArrayList<ArrayList<ArrayList<Integer[]>>> uavTimeframeSubPaths;
    private ArrayList<ArrayList<ArrayList<Integer[]>>> bssTimeframeSubPaths;
    private DARP darp;

    Map<String, Double> statisticsExecution;
    Map<String, Integer> statisticsMission;
    private List<List<Integer>> numCellsPerEachUavSubPath;
    private double totalElapsedTime;
    private double darpElapsedTime;
    private double stcElapsedTime;
    private double overallPPElapsedTime, uavPPElapsedTime, bssPPElapsedTime;
    private int numOfCoveredCells;
    private int minSubRegionSize, maxSubRegionSize;
    private int aStarTotalCost;
    private int aStarMinStepCost;
    private int aStarMaxStepCost;
    private int aStarStepCounter;

    private int startingUAVAutonomy;

    public WEAVE(Region region, ConsolePanel console)
            throws NoConnectedComponentException, WrongScenarioParametersException {
        this.console = console;

        statisticsExecution = new HashMap<>();
        statisticsMission = new HashMap<>();

        if (connectComponentChecking(region)) {
            throw new NoConnectedComponentException();
        }

        this.numOfDrones = region.getNumOfDrones();

        if (this.numOfDrones <= 0) {
            throw new WrongScenarioParametersException();
        }

        this.dronesSpecifications = new ParrotANAFI();
        this.chargingStationSpecifications = new BSS();
        this.region = region;
        this.rows = region.getMacroRows();
        this.cols = region.getMacroCols();
        this.numOfObstaclesCells = region.getMacroNumOfObstacles();

        this.droneRegion = region.getDroneMacroRegion();
        this.stationsRegion = region.getRoadsMicroCellWithStations();
        this.totalElapsedTime = 0;

        this.aStarTotalCost = 0;
        this.aStarStepCounter = 0;
        this.aStarMinStepCost = Integer.MAX_VALUE;
        this.aStarMaxStepCost = Integer.MIN_VALUE;
    }

    public void resolveProblem()
            throws DARPFailureException, UAVsPPNoIntersectionBetweenRoadAndDronePathException,
            BSSsPPFailureException, IncompatibleSimulatorVersionException, UAVsPPStopBadWeatherConditionsException,
            WrongScenarioParametersException {
        subroutineDARP(AppSettings.DARP_DISCREPANCY_CELLS, AppSettings.DARP_CC_VARIATION);
        subroutineSTC();
        subroutinePathPlanningWithWeatherForecasts();
        isSuccess = true;
    }

    private void subroutinePathPlanningWithWeatherForecasts()
            throws UAVsPPNoIntersectionBetweenRoadAndDronePathException, BSSsPPFailureException,
            UAVsPPStopBadWeatherConditionsException, WrongScenarioParametersException {
        int numBss = numOfDrones;
        int NUM_DEPOTS = 1;

        IBSS bssSpecification = new BSS();

        WeatherStation weatherStation = new WeatherStation(TEMPERATURE_DEFAULT);
        IUAV uav = new ParrotANAFI();

        int endForecastTime = (region.getMicroRows() * region.getMicroCols());
        int timeInterval = (int) Math.round(endForecastTime * .25);

        List<ConsumptionForecast> consumptionForecasts;
        if (IS_TEMPERATURE_VARIATION) {
            consumptionForecasts = uav
                    .getConsumptionForecasts(weatherStation.getWeatherForecasts(timeInterval, endForecastTime, 5));
        } else {
            consumptionForecasts = new ArrayList<>();
            consumptionForecasts.add(
                    new ConsumptionForecast(0, uav.getEffectiveCellPerMinutes(weatherStation.getStartTemperature())));
            this.startingUAVAutonomy = consumptionForecasts.get(0).getCellsPerBattery();
        }

        List<Coordinate> roadsCoordinates = getRoadsCoordinates(stationsRegion);

        long startTime = System.nanoTime();

        long startTimeSub = System.nanoTime();
        List<List<MovementUAV>> UAVsPathsWithTimestamp = uavPathPlanningWithWeatherForecasts(
                getPathSTC(),
                roadsCoordinates,
                consumptionForecasts,
                bssSpecification.getBatterySwappingTime());
        double uavPPElapsedTime = (double) (System.nanoTime() - startTimeSub) / Math.pow(10, 9);
        statisticsExecution.put(KEY_STATS_EXEC_PP_UAV, uavPPElapsedTime);

        NUM_DEPOTS = numBss;
        List<Coordinate> bssDepotsCoordinates = getBssDepotsCoordinates(NUM_DEPOTS, UAVsPathsWithTimestamp);

        List<MovementBSS> orderedPOS = UAVsPathsWithTimestamp.stream()
                .flatMap(List::stream)
                .filter(MovementUAV::isToCoordinatePOS)
                .map(e -> new MovementBSS(e.getTimestamp(), e.getToCoordinate()))
                .sorted(Comparator.comparing(MovementBSS::getTimestamp))
                .toList();

        startTimeSub = System.nanoTime();
        List<List<MovementBSS>> BSSsPathsWithTimestamp = bssPathPlanningWithWeatherForecasts(
                numBss,
                bssDepotsCoordinates,
                orderedPOS,
                bssSpecification.getBatterySwappingTime());
        double bssPPElapsedTime = (double) (System.nanoTime() - startTimeSub) / Math.pow(10, 9);
        statisticsExecution.put(KEY_STATS_EXEC_PP_BSS, bssPPElapsedTime);

        double overallPPElapsedTime = (double) (System.nanoTime() - startTime) / Math.pow(10, 9);
        totalElapsedTime += overallPPElapsedTime;
        statisticsExecution.put(KEY_STATS_EXEC_TOTAL, totalElapsedTime);

        int GUI_TIME_FRAME = 1;
        generateTimeframeForGUI(GUI_TIME_FRAME, UAVsPathsWithTimestamp, BSSsPathsWithTimestamp);

        collectMissionStatistics(
                bssSpecification.getBatterySwappingTime(),
                consumptionForecasts.stream().mapToInt(ConsumptionForecast::getCellsPerBattery).max().orElse(0),
                orderedPOS,
                UAVsPathsWithTimestamp,
                BSSsPathsWithTimestamp);

    }

    private List<List<MovementBSS>> bssPathPlanningWithWeatherForecasts(
            int numOfStation,
            List<Coordinate> depotsCoordinates,
            List<MovementBSS> pointsOfSwap,
            int batterySwapTime) throws BSSsPPFailureException {

        int[] lastTimestamp = new int[numOfStation];

        List<List<MovementBSS>> BSSsPathsWithTimestamp = new ArrayList<>();
        for (int s = 0; s < numOfStation; s++) {
            BSSsPathsWithTimestamp.add(new ArrayList<>());
            int assignedDepot = s % depotsCoordinates.size();
            BSSsPathsWithTimestamp.get(s).add(new MovementBSS(0, depotsCoordinates.get(assignedDepot)));
        }

        for (MovementBSS pos : pointsOfSwap) {

            boolean failure = false;

            for (int s = 0; s < numOfStation; s++) {

                int bssCurrentPositionArrayIndex = BSSsPathsWithTimestamp.get(s).size() - 1;
                MovementBSS bssCurrentMovement = BSSsPathsWithTimestamp.get(s).get(bssCurrentPositionArrayIndex);

                AStar infoViaggio = getTravelInfo(stationsRegion, bssCurrentMovement.getCoordinates(),
                        pos.getCoordinates());
                int timeToReachTarget = infoViaggio.totalCost;
                int actualTimestamp = bssCurrentMovement.getTimestamp();

                if (actualTimestamp + timeToReachTarget > pos.getTimestamp()) {
                    failure = true;

                } else {
                    failure = false;

                    List<Integer[]> pathToAdd = infoViaggio.getAStarPath().subList(1,
                            infoViaggio.getAStarPath().size());

                    for (Integer[] point : pathToAdd) {
                        lastTimestamp[s]++;
                        BSSsPathsWithTimestamp.get(s).add(new MovementBSS(lastTimestamp[s], new Coordinate(point)));
                    }
                    lastTimestamp[s] = pos.getTimestamp();
                    lastTimestamp[s] += batterySwapTime;
                    BSSsPathsWithTimestamp.get(s).add(new MovementBSS(lastTimestamp[s], pos.getCoordinates()));

                    break;
                }

            }

            if (failure) {
                String msg = "Nessuna stazione riesce a coprire il punto " + pos;
                System.err.println(msg);
                throw new BSSsPPFailureException(msg);
            }

        }

        return BSSsPathsWithTimestamp;
    }

    private List<List<MovementUAV>> uavPathPlanningWithWeatherForecasts(
            List<List<MovementUAV>> UAVsPathsSTC,
            List<Coordinate> roadsCoordinates,
            List<ConsumptionForecast> consumptionForecasts,
            int batterySwapTime)
            throws UAVsPPNoIntersectionBetweenRoadAndDronePathException, UAVsPPStopBadWeatherConditionsException {
        int CELL_VISIT_TIME = 1;

        List<List<MovementUAV>> UAVsPathsWithTimestamp = new ArrayList<>();

        for (int k = 0; k < numOfDrones; k++) {

            Coordinate depotCoordinates = UAVsPathsSTC.get(k).get(0).getFromCoordinate();

            Set<Coordinate> BSPs = getBatterySwappingPoints(UAVsPathsSTC.get(k), roadsCoordinates, depotCoordinates);
            List<MovementUAV> workingPath = new ArrayList<>(UAVsPathsSTC.get(k));
            List<MovementUAV> fixedPath = new ArrayList<>();

            for (MovementUAV movement : workingPath) {
                movement.setFromCoordinateBSP(BSPs.contains(movement.getFromCoordinate()));
                movement.setToCoordinateBSP(BSPs.contains(movement.getToCoordinate()));
            }

            int prevEnergy = 0;

            for (ConsumptionForecast forecast : consumptionForecasts) {

                int timestamp = forecast.getTimestamp();
                int energy = forecast.getCellsPerBattery();

                System.out.println("### TIMESTAMP " + timestamp + " | UAV " + k + " | Cells=" + energy + " ###");

                MovementUAV startingMovement = null;
                MovementUAV lastPOS = null;

                int cellsBetweenStartingMovementAndLastPOS = -1;
                int cellsFromDeposit = 0;
                Iterator<MovementUAV> pathIterator = workingPath.iterator();
                while (pathIterator.hasNext()) {
                    MovementUAV movement = pathIterator.next();
                    if (movement.getTimestamp() < timestamp) {
                        cellsFromDeposit++;
                        fixedPath.add(movement);
                        pathIterator.remove();
                        if (movement.isToCoordinatePOS()) {
                            lastPOS = movement;
                        }
                        if (movement.isFromCoordinatePOS()) {
                            cellsBetweenStartingMovementAndLastPOS = 0;
                        }
                        if (cellsBetweenStartingMovementAndLastPOS != -1)
                            cellsBetweenStartingMovementAndLastPOS++;
                    } else {
                        cellsFromDeposit++;
                        startingMovement = movement;
                        if (movement.isToCoordinatePOS()) {
                            lastPOS = movement;
                        }
                        if (movement.isFromCoordinatePOS()) {
                            cellsBetweenStartingMovementAndLastPOS = 0;
                        }
                        break;
                    }

                }

                if (startingMovement == null) {
                    System.out.println("Ho finito il percorso prima di scorrere tutte le previsioni.");
                    continue;

                }

                BSPs = getBatterySwappingPoints(workingPath, roadsCoordinates, depotCoordinates);

                int energyDelta = energy - prevEnergy;
                int startingMovementEnergy;

                if (timestamp == 0) {
                    startingMovementEnergy = energy;
                }

                else if (lastPOS == null) {
                    startingMovementEnergy = energy - cellsFromDeposit;
                }

                else if (startingMovement.isToCoordinatePOS()) {
                    startingMovementEnergy = 1;
                }

                else if (startingMovement.isFromCoordinatePOS()) {
                    startingMovementEnergy = energy;
                }

                else {
                    if (energyDelta <= 0) {

                        startingMovementEnergy = energy - cellsBetweenStartingMovementAndLastPOS;
                    } else {

                        startingMovementEnergy = (prevEnergy - cellsBetweenStartingMovementAndLastPOS) + energyDelta;
                    }
                }

                if ((startingMovementEnergy < 0)
                        || (startingMovementEnergy == 0 && !startingMovement.isToCoordinateBSP())) {
                    Coordinate puntoDiArresto;
                    if (lastPOS == null)
                        puntoDiArresto = depotCoordinates;
                    else
                        puntoDiArresto = lastPOS.getToCoordinate();
                    String exceptionMessage = "Per condizioni meteo avverse al tempo " + timestamp +
                            ", UAV " + k + " si arresta in " + puntoDiArresto;
                    System.err.println(exceptionMessage);
                    throw new UAVsPPStopBadWeatherConditionsException(exceptionMessage);
                }

                for (MovementUAV movementUAV : workingPath) {
                    movementUAV.setToCoordinatePOS(false);
                    movementUAV.setFromCoordinatePOS(false);
                }

                Map<Coordinate, List<MovementUAV>> checkPointMaxSubPath = getUAVsMaxSubPathsOptimized(
                        workingPath,
                        BSPs,
                        startingMovement,
                        depotCoordinates,
                        energy,
                        startingMovementEnergy);

                Set<Coordinate> chosenCheckPoints = new HashSet<>();
                boolean isPathExisting = existPath(
                        startingMovement.getFromCoordinate(),
                        depotCoordinates,
                        checkPointMaxSubPath,
                        chosenCheckPoints);

                if (!isPathExisting) {
                    Coordinate puntoDiArresto;
                    if (lastPOS == null)
                        puntoDiArresto = depotCoordinates;
                    else
                        puntoDiArresto = lastPOS.getToCoordinate();
                    String msg = "Nessuna soluzione. ";
                    if (energyDelta < 0) {
                        msg += "A causa della variazione delle condizioni atmosferiche al tempo " + timestamp +
                                " UAV " + k + " non riuscirebbe a completare il percorso. Termina in " + puntoDiArresto;
                    } else {
                        int motivation = getMotivation(workingPath, BSPs);
                        msg += "Per il drone " + k + ", serve una capacità della batteria di almeno Q=" + motivation;
                    }
                    System.err.println(msg);
                    throw new UAVsPPNoIntersectionBetweenRoadAndDronePathException(msg);
                }

                Set<Coordinate> pos = new HashSet<>(chosenCheckPoints);
                pos.remove(depotCoordinates);
                if (lastPOS != null) {
                    Coordinate lastPos = lastPOS.getToCoordinate();
                    if (!lastPos.equals(startingMovement.getFromCoordinate()))
                        pos.remove(startingMovement.getFromCoordinate());
                } else {
                    pos.remove(startingMovement.getFromCoordinate());
                    pos.remove(startingMovement.getToCoordinate());
                }

                for (MovementUAV m : workingPath) {
                    m.setToCoordinatePOS(pos.contains(m.getToCoordinate()));
                    m.setFromCoordinatePOS(pos.contains(m.getFromCoordinate()));
                }

                int deltaTimestamp = 0;
                if (lastPOS != null)
                    deltaTimestamp = batterySwapTime - ((timestamp - 1) - lastPOS.getTimestamp());

                int startingMovementTimestamp;
                if (timestamp == 0)
                    startingMovementTimestamp = 1;
                else if (startingMovement.isFromCoordinatePOS())
                    startingMovementTimestamp = timestamp + deltaTimestamp;
                else
                    startingMovementTimestamp = timestamp;

                workingPath = calculateTimestampForUAVPath(
                        workingPath,
                        startingMovementTimestamp,
                        CELL_VISIT_TIME,
                        batterySwapTime);

                prevEnergy = energy;
            }

            fixedPath.addAll(workingPath);
            UAVsPathsWithTimestamp.add(fixedPath);

        }

        return UAVsPathsWithTimestamp;
    }

    private List<MovementUAV> calculateTimestampForUAVPath(List<MovementUAV> path, int timestamp, int cellVisitTime,
            int batterySwapTime) {
        List<MovementUAV> pathWithTimestamp = new ArrayList<>(path);
        pathWithTimestamp.get(0).setTimestamp(timestamp);
        for (int i = 1; i < pathWithTimestamp.size(); i++) {
            MovementUAV movement = pathWithTimestamp.get(i);

            timestamp += cellVisitTime;
            if (movement.isFromCoordinatePOS()) {
                timestamp += batterySwapTime;
            }
            movement.setTimestamp(timestamp);

        }
        return pathWithTimestamp;
    }

    private Map<Coordinate, List<MovementUAV>> getUAVsMaxSubPathsOptimized(
            List<MovementUAV> trajectory,
            Set<Coordinate> BSPs,
            MovementUAV startingMovement,
            Coordinate depotCoordinate,
            int energy,
            int startingEnergy) {
        boolean isStartingMovement = true;

        Set<Coordinate> checkPointsPartenzaSet = new HashSet<>(BSPs);
        Coordinate startingCoordinate = startingMovement.getFromCoordinate();

        checkPointsPartenzaSet.remove(startingCoordinate);
        List<Coordinate> checkPointsPartenza = new ArrayList<>(checkPointsPartenzaSet);

        checkPointsPartenza.add(0, startingCoordinate);

        Set<Coordinate> checkPointsArrivo = new HashSet<>(checkPointsPartenza);
        checkPointsArrivo.add(depotCoordinate);

        Map<Coordinate, Integer> indexOf = new HashMap<>();
        for (int i = 0; i < trajectory.size(); i++) {
            indexOf.put(trajectory.get(i).getFromCoordinate(), i);
        }

        Map<Coordinate, List<MovementUAV>> maxPathFromCheckPoint = new HashMap<>();

        for (Coordinate checkPoint : checkPointsPartenza) {

            if (Objects.equals(checkPoint, new Coordinate(3, 3))) {
                System.out.println();
            }

            int startingPointIndex = indexOf.get(checkPoint);

            int maxMoveCells;
            if (isStartingMovement) {
                maxMoveCells = startingEnergy;
                isStartingMovement = false;
            } else
                maxMoveCells = energy;

            for (int motion = maxMoveCells - 1; motion >= 0; motion--) {

                int maxReachablePointIndex = startingPointIndex + motion;
                if (maxReachablePointIndex >= trajectory.size())
                    continue;

                if (checkPointsArrivo.contains(trajectory.get(maxReachablePointIndex).getToCoordinate())) {
                    maxPathFromCheckPoint.put(checkPoint,
                            trajectory.subList(startingPointIndex, maxReachablePointIndex + 1));
                    break;
                }
            }
        }

        return maxPathFromCheckPoint;

    }

    private List<Coordinate> getBssDepotsCoordinates(
            int numOfDepots,
            List<List<MovementUAV>> UAVsPathsWithTimestamp) throws WrongScenarioParametersException {

        if (numOfDepots > numOfDrones) {
            String msg = "Numero di depots " + numOfDepots + " " +
                    "maggiore del numero di droni " + numOfDrones;
            System.err.println(msg);
            throw new WrongScenarioParametersException(msg);
        }

        List<Coordinate> depotsCoordinates = new ArrayList<>();
        for (int d = 0; d < numOfDrones; d++) {
            Coordinate uavStartingPoint = UAVsPathsWithTimestamp.get(d).get(0).getFromCoordinate();
            int ratio = numOfDrones / numOfDepots;
            if (d % ratio == 0) {
                depotsCoordinates.add(uavStartingPoint);
            }
        }
        return depotsCoordinates;
    }

    private List<List<MovementUAV>> getPathSTC() {
        List<List<MovementUAV>> UAVsPathsWithTimestamp = new ArrayList<>();
        for (int d = 0; d < numOfDrones; d++) {
            UAVsPathsWithTimestamp.add(new ArrayList<>());
            for (int i = 0; i < dronesPaths.get(d).size(); i++) {
                Integer[] step = dronesPaths.get(d).get(i);
                Integer[] from = { step[0], step[1] };
                Integer[] to = { step[2], step[3] };
                UAVsPathsWithTimestamp.get(d).add(new MovementUAV(0, new Coordinate(from), new Coordinate(to)));
            }

            Integer[] step = dronesPaths.get(d).get(dronesPaths.get(d).size() - 1);
            Integer[] from = { step[2], step[3] };
            step = dronesPaths.get(d).get(0);
            Integer[] to = { step[0], step[1] };
            UAVsPathsWithTimestamp.get(d).add(new MovementUAV(0, new Coordinate(from), new Coordinate(to)));
        }
        return UAVsPathsWithTimestamp;
    }

    private void collectMissionStatistics(int tempoSwap, int maxUavCells, List<MovementBSS> effectivePointsOfSwap,
            List<List<MovementUAV>> uavsPathsWithSwappingTimestamp,
            List<List<MovementBSS>> pathsWithTimestampsForEachBSS) {

        int numeroBSS = (int) pathsWithTimestampsForEachBSS.stream()
                .filter(e -> e.size() > 1)
                .count();

        int tempoMissionBSS = pathsWithTimestampsForEachBSS.stream()
                .map(List::size)
                .filter(size -> size > 1)
                .max(Integer::compareTo)
                .orElse(0);

        numCellsPerEachUavSubPath = getNumCellsPerEachUavSubPath(uavsPathsWithSwappingTimestamp, effectivePointsOfSwap);

        int cumulativeWastedEnergy = numCellsPerEachUavSubPath.stream()
                .map(uavCellsPath -> uavCellsPath.subList(0, uavCellsPath.size() - 1))
                .flatMap(List::stream)
                .mapToInt(numCells -> maxUavCells - numCells)
                .sum();

        int cumulativeEnergy = cumulativeWastedEnergy + numCellsPerEachUavSubPath.stream()
                .flatMap(List::stream)
                .mapToInt(Integer::intValue)
                .sum();

        int cumulativeBatteryUsed = numCellsPerEachUavSubPath.stream()
                .mapToInt(List::size)
                .sum();

        int cumulativeSwappingTime = (cumulativeBatteryUsed - numOfDrones) * tempoSwap;
        int uavMissionTime = uavsPathsWithSwappingTimestamp.stream()
                .map(uavPath -> uavPath.get(uavPath.size() - 1).getTimestamp())
                .max(Integer::compareTo)
                .orElse(0);

        statisticsMission.put(KEY_STATS_MISSION_STARTING_UAV_AUTONOMY, startingUAVAutonomy);

        statisticsMission.put(KEY_STATS_MISSION_NUM_UAV, numOfDrones);
        statisticsMission.put(KEY_STATS_MISSION_NUM_BSS, numeroBSS);

        statisticsMission.put(KEY_STATS_MISSION_CUMUL_UAV_ENERGY, cumulativeEnergy);
        statisticsMission.put(KEY_STATS_MISSION_CUMUL_UAV_WASTE, cumulativeWastedEnergy);
        statisticsMission.put(KEY_STATS_MISSION_CUMUL_UAV_BATTERIES, cumulativeBatteryUsed);

        statisticsMission.put(KEY_STATS_MISSION_TIME, uavMissionTime);
        statisticsMission.put(KEY_STATS_MISSION_BSS_TIME, tempoMissionBSS);
        statisticsMission.put(KEY_STATS_MISSION_CUMUL_SWAP_TIME, cumulativeSwappingTime);
    }

    private void generateTimeframeForGUI(int TIME_FRAME,
            List<List<MovementUAV>> uavsPathsWithSwappingTimestamp,
            List<List<MovementBSS>> pathsWithTimestampsForEachBSS) {

        this.uavTimeframeSubPaths = new ArrayList<>();
        this.bssTimeframeSubPaths = new ArrayList<>();

        for (int d = 0; d < numOfDrones; d++) {

            ArrayList<ArrayList<Integer[]>> singleUAVSUBPATHS = new ArrayList<>();
            ArrayList<ArrayList<Integer[]>> bssSubPaths = new ArrayList<>();

            int lowerTimeframe = 0;
            int upperTimeframe = TIME_FRAME;
            int uavSubPathIndex = 0;
            singleUAVSUBPATHS.add(new ArrayList<>());
            List<MovementUAV> uavSubPath = uavsPathsWithSwappingTimestamp.get(d);
            Iterator<MovementUAV> uavSubPathIterator = uavSubPath.iterator();

            boolean isMoveToNext = true;
            MovementUAV movement = null;
            while (uavSubPathIterator.hasNext()) {
                if (isMoveToNext)
                    movement = uavSubPathIterator.next();

                if (lowerTimeframe <= movement.getTimestamp() && movement.getTimestamp() < upperTimeframe) {
                    singleUAVSUBPATHS.get(uavSubPathIndex).add(movement.getSequence());
                    isMoveToNext = true;
                } else {
                    uavSubPathIndex++;
                    singleUAVSUBPATHS.add(new ArrayList<>());
                    isMoveToNext = false;
                    lowerTimeframe = upperTimeframe;
                    upperTimeframe += TIME_FRAME;
                }
            }

            lowerTimeframe = 0;
            upperTimeframe = TIME_FRAME;
            int bssSubPathIndex = 0;
            bssSubPaths.add(new ArrayList<>());
            List<MovementBSS> bssPaths = pathsWithTimestampsForEachBSS.get(d);
            Iterator<MovementBSS> bssPathsIterator = bssPaths.iterator();

            isMoveToNext = true;
            MovementBSS movementBSS = null;
            while (bssPathsIterator.hasNext()) {
                if (isMoveToNext)
                    movementBSS = bssPathsIterator.next();

                if (lowerTimeframe <= movementBSS.getTimestamp() && movementBSS.getTimestamp() < upperTimeframe) {
                    bssSubPaths.get(bssSubPathIndex).add(movementBSS.getCoordinates().getList());
                    isMoveToNext = true;
                } else {
                    bssSubPathIndex++;
                    bssSubPaths.add(new ArrayList<>());
                    isMoveToNext = false;
                    lowerTimeframe = upperTimeframe;
                    upperTimeframe += TIME_FRAME;
                }
            }

            if (singleUAVSUBPATHS.size() > bssSubPaths.size()) {
                int gap = singleUAVSUBPATHS.size() - bssSubPaths.size();
                for (int i = 0; i < gap; i++) {
                    bssSubPaths.add(new ArrayList<>());
                }
            } else if (bssSubPaths.size() > singleUAVSUBPATHS.size()) {
                int gap = bssSubPaths.size() - singleUAVSUBPATHS.size();
                for (int i = 0; i < gap; i++) {
                    singleUAVSUBPATHS.add(new ArrayList<>());
                }
            }

            uavTimeframeSubPaths.add(singleUAVSUBPATHS);
            bssTimeframeSubPaths.add(bssSubPaths);

        }
    }

    private List<List<Integer>> getNumCellsPerEachUavSubPath(List<List<MovementUAV>> uavsPaths,
            List<MovementBSS> effectivePointsOfSwap) {

        Set<Coordinate> bsps = effectivePointsOfSwap.stream()
                .map(MovementBSS::getCoordinates)
                .collect(Collectors.toSet());

        List<List<Integer>> numCellsPerEachUavSubPath = new ArrayList<>();

        for (int i = 0; i < uavsPaths.size(); i++) {

            numCellsPerEachUavSubPath.add(new ArrayList<>());

            List<MovementUAV> uavPath = uavsPaths.get(i);
            List<Integer> numCellForUav = numCellsPerEachUavSubPath.get(i);

            int counter = 0;
            for (MovementUAV e : uavPath) {
                if (bsps.contains(e.getFromCoordinate())) {
                    numCellForUav.add(counter);
                    counter = 0;
                }
                counter++;
            }
            int lastSequence = uavPath.size() - numCellForUav.stream().mapToInt(e -> e).sum();
            numCellForUav.add(lastSequence);
        }

        return numCellsPerEachUavSubPath;
    }

    private AStar getTravelInfo(int[][] roads, Coordinate startCoordinates, Coordinate endCoordinates) {
        AStar a = new AStar(
                roads,

                TypeConverter.wrapperToPrimitive(endCoordinates.getList()),
                TypeConverter.wrapperToPrimitive(startCoordinates.getList()));
        a.start();
        return a;
    }

    private int getMotivation(List<MovementUAV> paths, Set<Coordinate> checkPoints) {
        int minQ = 0;
        int counterQ = 0;

        List<MovementUAV> editedPaths = new ArrayList<>(paths);
        editedPaths.add(editedPaths.get(0));

        for (MovementUAV editedPath : editedPaths) {
            counterQ++;
            if (checkPoints.contains(editedPath.getFromCoordinate())) {
                if (counterQ > minQ) {
                    minQ = counterQ;
                }
                counterQ = 0;
            }
        }

        return minQ;
    }

    private boolean existPath(
            Coordinate start,
            Coordinate target,
            Map<Coordinate, List<MovementUAV>> checkPointMaxSubPath,
            Set<Coordinate> POS) {

        List<MovementUAV> subPath;
        if (checkPointMaxSubPath.containsKey(start) && checkPointMaxSubPath.get(start).size() > 0) {
            subPath = checkPointMaxSubPath.get(start);
        } else {
            return false;
        }

        Coordinate lastNodeOfPath = subPath.get(subPath.size() - 1).getToCoordinate();

        if (lastNodeOfPath.equals(target)) {
            POS.add(subPath.get(0).getFromCoordinate());
            return true;
        } else {
            boolean result = existPath(lastNodeOfPath, target, checkPointMaxSubPath, POS);
            if (result) {
                POS.add(subPath.get(0).getFromCoordinate());
                return true;
            }
            return false;
        }
    }

    private Set<Coordinate> getBatterySwappingPoints(
            List<MovementUAV> uavPathSTC,
            List<Coordinate> roadsCoordinates,
            Coordinate depotCoordinates) {

        Set<Coordinate> coordinates = new HashSet<>();
        Set<Coordinate> roads = new HashSet<>(roadsCoordinates);

        if (uavPathSTC.size() > 0) {
            Coordinate destinationCoordinates = uavPathSTC.get(0).getFromCoordinate();
            if (roads.contains(destinationCoordinates)) {
                coordinates.add(destinationCoordinates);
            }
        }

        for (MovementUAV movement : uavPathSTC) {
            Coordinate destinationCoordinates = movement.getToCoordinate();
            if (roads.contains(destinationCoordinates)) {
                coordinates.add(destinationCoordinates);
            }
        }

        coordinates.remove(depotCoordinates);

        return coordinates;
    }

    public void printSubPath(ArrayList<Integer[]> subPath, String title) {
        System.out.printf("%s (size %d): ", title, subPath.size());
        for (Integer[] c : subPath)
            System.out.print(Arrays.toString(c) + " ");
        System.out.println();
    }

    public void printAllPaths(ArrayList<ArrayList<ArrayList<Integer[]>>> paths, String title) {
        System.out.println(title);
        for (int i = 0; i < paths.size(); i++) {
            ArrayList<ArrayList<Integer[]>> path = paths.get(i);
            System.out.printf("### PATH %d ###\n", i);
            for (ArrayList<Integer[]> subPath : path) {
                System.out.print("{");
                for (Integer[] step : subPath) {
                    System.out.print(Arrays.toString(step));
                }
                System.out.print("} ");
            }
            System.out.println();
        }
    }

    private ArrayList<Integer[]> getSubPathStartEndCoordinates(ArrayList<Integer[]> path, int startIndex, int endIndex,
            boolean isLastStep) {
        ArrayList<Integer[]> subPath = new ArrayList<>();
        for (int i = startIndex; i < endIndex; i++) {
            subPath.add(path.get(i).clone());
        }
        if (isLastStep)
            subPath.add(path.get(endIndex).clone());
        return subPath;
    }

    private ArrayList<Integer[]> calculateStationPath(int[][] road, Integer[] startCoordinates,
            Integer[] endCoordinates) {
        AStar a = new AStar(road, TypeConverter.wrapperToPrimitive(endCoordinates),
                TypeConverter.wrapperToPrimitive(startCoordinates));
        a.start();
        updateAStarStatistics(a.getTotalCost());
        return a.aStarPath;
    }

    private void updateAStarStatistics(int cost) {
        this.aStarTotalCost += cost;
        this.aStarStepCounter++;

        if (cost > aStarMaxStepCost)
            aStarMaxStepCost = cost;
        if (cost < aStarMinStepCost)
            aStarMinStepCost = cost;
    }

    private int getIndexOfCoordinate(ArrayList<Integer[]> path, Integer[] coordinates) {
        int index = -1;
        for (int i = 0; i < path.size(); i++) {
            int y1 = path.get(i)[0];
            int x1 = path.get(i)[1];
            int y2 = coordinates[0];
            int x2 = coordinates[1];

            if (y1 == y2 && x1 == x2)
                return i;
        }
        return index;
    }

    private Integer[] getMaxIntersection(int[][] road, ArrayList<Integer[]> maxDronePath) {
        Integer[] maxIntersection = null;
        for (Integer[] c : maxDronePath) {
            if (road[c[0]][c[1]] == 0) {
                maxIntersection = c.clone();
            }
        }
        return maxIntersection;
    }

    private ArrayList<Integer[]> getSubPath(ArrayList<Integer[]> path, int startIndex, int endIndex) {
        ArrayList<Integer[]> subPath = new ArrayList<>();
        subPath.add(getStartCoordinate(startIndex, path));
        for (int i = startIndex; i < endIndex; i++) {
            subPath.add(getEndCoordinate(i, path));
        }
        return subPath;
    }

    private Integer[] getStartCoordinate(int index, ArrayList<Integer[]> dronePath) {
        return new Integer[] { dronePath.get(index)[0], dronePath.get(index)[1] };
    }

    private Integer[] getEndCoordinate(int index, ArrayList<Integer[]> dronePath) {
        return new Integer[] { dronePath.get(index)[2], dronePath.get(index)[3] };
    }

    private void subroutineDARP(int darpDiscrepancyCells, double darpCCVariation) throws DARPFailureException {

        boolean customDroneAllocation = false;
        int numOfUAV = numOfDrones;

        double[] droneAreaPortion = new double[numOfUAV];
        if (!customDroneAllocation) {
            for (int i = 0; i < numOfUAV; i++) {
                droneAreaPortion[i] = 1.0 / numOfUAV;
            }
        }

        else {
            droneAreaPortion[0] = 0.1;
            droneAreaPortion[1] = 0.3;
            droneAreaPortion[2] = 0.6;
        }

        darp = new DARP(
                rows,
                cols,
                droneRegion,
                AppSettings.DARP_MAX_ITER,
                darpCCVariation,
                AppSettings.DARP_RANDOM_LEVEL,
                darpDiscrepancyCells,
                AppSettings.DARP_IMPORTANCE,
                droneAreaPortion);

        darp.constructAssignmentM();

        if (darp.getSuccess()) {
            if (darp.getAchievedDiscr() < 2) {
                console.info(AppStrings.DARP_OPTIMAL_DIVISION);
            } else {
                console.info(AppStrings.DARP_ACCEPTABLE_DIVISION);
            }
        } else {
            throw new DARPFailureException(String.format(AppStrings.DARP_FAILURE_EXCEPTION_MESSAGE,
                    darp.getElapsedTime(), darp.getTotalIterations(), 4 * darp.getDiscr()));
        }

        this.assignmentMatrix = darp.getAssignmentMatrix();
        this.dronePositionMatrix = darp.getRobotBinary();
        this.dronesRegions = darp.getBinaryRobotRegions();
        this.startingPositions = darp.getDroneInitialPosition();
        this.darpElapsedTime = darp.getElapsedTime();
        this.numOfCoveredCells = darp.getEffectiveSize();
        this.minSubRegionSize = darp.getMinCellsAss();
        this.maxSubRegionSize = darp.getMaxCellsAss();

        statisticsExecution.put(KEY_STATS_EXEC_DARP, darpElapsedTime);
        this.totalElapsedTime += darpElapsedTime;
    }

    private void subroutineSTC() {
        long startTime = System.nanoTime();

        final int numOfMSTmodes = 4;

        ArrayList<ArrayList<Integer[]>>[][] droneTrajectories = getDroneTrajectories(numOfMSTmodes);

        int[][] roadMatrix = region.getRoadsMicroCellWithStations();

        dronesPaths = new ArrayList<>();
        for (int k = 0; k < numOfDrones; k++) {

            double[] scores = new double[numOfMSTmodes];
            for (int i = 0; i < numOfMSTmodes; i++) {
                int[][] line = getTypesOfLines(droneTrajectories[k][i]);
                scores[i] = calculateScoreMST(roadMatrix, line, i);

            }

            int indexOfMaxScore = getIndexOfMaxValue(scores);

            dronesPaths.add(droneTrajectories[k][indexOfMaxScore].get(0));
        }

        double stcElapsedTime = (double) (System.nanoTime() - startTime) / Math.pow(10, 9);
        statisticsExecution.put(KEY_STATS_EXEC_STC, stcElapsedTime);
        totalElapsedTime += stcElapsedTime;
    }

    private int[][] getTypesOfLines(ArrayList<ArrayList<Integer[]>> arrayLists) {
        int[][][] TypesOfLines = new int[region.getMacroRows() * 2][region.getMacroCols() * 2][2];

        int indxadd1;
        int indxadd2;

        boolean flag = false;
        for (Integer[] connection : arrayLists.get(0)) {
            if (flag) {
                if (TypesOfLines[connection[0]][connection[1]][0] == 0) {
                    indxadd1 = 0;
                } else {
                    indxadd1 = 1;
                }
                if (TypesOfLines[connection[2]][connection[3]][0] == 0 && flag) {
                    indxadd2 = 0;
                } else {
                    indxadd2 = 1;
                }
            } else {
                if (!(TypesOfLines[connection[0]][connection[1]][0] == 0)) {
                    indxadd1 = 0;
                } else {
                    indxadd1 = 1;
                }
                if (!(TypesOfLines[connection[2]][connection[3]][0] == 0 && flag)) {
                    indxadd2 = 0;
                } else {
                    indxadd2 = 1;
                }
            }

            flag = true;

            if (connection[0].equals(connection[2])) {
                if (connection[1] > connection[3]) {
                    TypesOfLines[connection[0]][connection[1]][indxadd1] = 2;
                    TypesOfLines[connection[2]][connection[3]][indxadd2] = 3;
                } else {
                    TypesOfLines[connection[0]][connection[1]][indxadd1] = 3;
                    TypesOfLines[connection[2]][connection[3]][indxadd2] = 2;
                }
            } else {
                if (connection[0] > connection[2]) {
                    TypesOfLines[connection[0]][connection[1]][indxadd1] = 1;
                    TypesOfLines[connection[2]][connection[3]][indxadd2] = 4;
                } else {
                    TypesOfLines[connection[0]][connection[1]][indxadd1] = 4;
                    TypesOfLines[connection[2]][connection[3]][indxadd2] = 1;
                }
            }
        }

        int[][] lines = new int[region.getMicroRows()][region.getMicroCols()];

        for (int i = 0; i < TypesOfLines.length; i++) {
            for (int j = 0; j < TypesOfLines[0].length; j++) {
                lines[i][j] = TypesOfLines[i][j][0];

            }

        }

        return lines;
    }

    private ArrayList<ArrayList<Integer[]>>[][] getDroneTrajectories(int numOfDirections) {
        ArrayList<ArrayList<Integer[]>>[][] droneTrajectories = new ArrayList[numOfDrones][numOfDirections];
        for (int k = 0; k < numOfDrones; k++)
            for (int direction = 0; direction < numOfDirections; direction++)
                droneTrajectories[k][direction] = new ArrayList<>();

        for (int mode = 0; mode < 4; mode++) {

            kruskalMSTs = calculateMSTs(dronesRegions, numOfDrones, region.getMacroRows(), region.getMacroCols(), mode);

            for (int d = 0; d < numOfDrones; d++) {

                CalculateTrajectories ct = new CalculateTrajectories(rows, cols, kruskalMSTs.get(d));

                ct.initializeGraph(region.getMicroDroneRegions(dronesRegions.get(d)), true);
                ct.RemoveTheAppropriateEdges();
                int startingNode = 4 * startingPositions.get(d)[0] * cols + 2 * startingPositions.get(d)[1];
                ct.CalculatePathsSequence(startingNode);
                droneTrajectories[d][mode].add(ct.getPathSequence());
            }
        }
        return droneTrajectories;
    }

    private ArrayList<Vector<?>> calculateMSTs(ArrayList<boolean[][]> binaryDroneRegions, int numOfDrones,
            int rows, int cols, int mode) {
        ArrayList<Vector<?>> MSTs = new ArrayList<>();
        for (int d = 0; d < numOfDrones; d++) {
            Kruskal k = new Kruskal(rows * cols);
            k.initializeGraph(binaryDroneRegions.get(d), true, mode);
            k.performKruskal();
            MSTs.add(k.getAllNewEdges());
        }
        return MSTs;
    }

    private List<Coordinate> getRoadsCoordinates(int[][] roads) {
        List<Coordinate> roadsCoordinates = new ArrayList<>();
        int ROAD = 0;

        assert roads.length == roads[0].length;
        for (int i = 0; i < roads.length; i++) {
            for (int j = 0; j < roads.length; j++) {
                if (roads[i][j] == ROAD) {
                    roadsCoordinates.add(new Coordinate(i, j));
                }
            }
        }

        return roadsCoordinates;
    }

    private boolean connectComponentChecking(Region region) {
        ConnectComponent cc = new ConnectComponent();
        cc.compactLabeling(
                region.getUnoccupiedCellMacroRegionMatrix(),
                new Dimension(region.getMacroCols(), region.getMacroRows()),
                true);
        return cc.getMaxLabel() > 1;
    }

    public int[][] getAssignmentMatrix() {
        return assignmentMatrix;
    }

    public ArrayList<boolean[][]> getDronesRegions() {
        return dronesRegions;
    }

    public boolean[][] getDronePositionMatrix() {
        return dronePositionMatrix;
    }

    public ArrayList<Integer[]> getStartingPositions() {
        return startingPositions;
    }

    public double getTotalElapsedTime() {
        return totalElapsedTime;
    }

    public double getDarpElapsedTime() {
        return darpElapsedTime;
    }

    public int getNumOfCoveredCells() {
        return numOfCoveredCells;
    }

    public int getEffectiveRegionSize() {
        return this.rows * this.cols * 4;
    }

    public int getMinSubRegionSize() {
        return minSubRegionSize;
    }

    public int getMaxSubRegionSize() {
        return maxSubRegionSize;
    }

    public int getNumOfDrones() {
        return numOfDrones;
    }

    public int getEffectiveNumOfObstacles() {
        return numOfObstaclesCells * 4;
    }

    public boolean isAbort() {
        return isAbort;
    }

    public void setAbort(boolean abort) {
        darp.setCanceled(true);
        isAbort = abort;
    }

    public boolean isSuccess() {
        return isSuccess;
    }

    public double getStcElapsedTime() {
        return stcElapsedTime;
    }

    public double getPathPlanningElapsedTime() {
        return overallPPElapsedTime;
    }

    public int getMaxPathLen() {
        int max = Integer.MIN_VALUE;
        for (ArrayList<Integer[]> dronePath : dronesPaths)
            if (max < dronePath.size())
                max = dronePath.size();
        return max;
    }

    public int getMinPathLen() {
        int min = Integer.MAX_VALUE;
        for (ArrayList<Integer[]> dronePath : dronesPaths)
            if (dronePath.size() < min)
                min = dronePath.size();
        return min;
    }

    public ArrayList<ArrayList<Integer[]>> getDronesPaths() {
        return dronesPaths;
    }

    public ArrayList<Vector<?>> getKruskalMSTs() {
        return kruskalMSTs;
    }

    public ArrayList<ArrayList<ArrayList<Integer[]>>> getUavTimeframeSubPaths() {
        return uavTimeframeSubPaths;
    }

    public ArrayList<ArrayList<ArrayList<Integer[]>>> getBssTimeframeSubPaths() {
        return bssTimeframeSubPaths;
    }

    public double getAStarTotalCost() {
        return (double) aStarTotalCost / (double) 10;
    }

    public double getAStarStepMaxCost() {
        return (double) aStarMaxStepCost / (double) 10;
    }

    public double getAStartStepMinCost() {
        return (double) aStarMinStepCost / (double) 10;
    }

    public double getAStarStepAvgCost() {
        return getAStarTotalCost() / (double) aStarStepCounter;
    }

    public int getDronesCompleteCoverageTime() {
        return (getMaxPathLen() / dronesSpecifications.getCellPerSeconds()) + getBatterySwappingTime();
    }

    public int getBatterySwappingTime() {
        return chargingStationSpecifications.getBatterySwappingTime() * (getMaxSubPathSteps() - 1);
    }

    public int getMaxSubPathSteps() {
        int maxStep = Integer.MIN_VALUE;
        for (ArrayList<ArrayList<Integer[]>> path : uavTimeframeSubPaths) {
            if (path.size() > maxStep) {
                maxStep = path.size();
            }
        }
        return maxStep;
    }

    public int getMinSubPathSteps() {
        int minStep = Integer.MAX_VALUE;
        for (ArrayList<ArrayList<Integer[]>> path : uavTimeframeSubPaths) {
            if (minStep > path.size()) {
                minStep = path.size();
            }
        }
        return minStep;
    }

    public int getMinSubPathLen() {
        int minLen = Integer.MAX_VALUE;
        for (ArrayList<ArrayList<Integer[]>> path : uavTimeframeSubPaths)
            for (ArrayList<Integer[]> subPath : path)
                if (minLen > subPath.size())
                    minLen = subPath.size();
        return minLen;
    }

    public int getMaxSubPathLen() {
        int maxLen = Integer.MIN_VALUE;
        for (ArrayList<ArrayList<Integer[]>> path : uavTimeframeSubPaths)
            for (ArrayList<Integer[]> subPath : path)
                if (subPath.size() > maxLen)
                    maxLen = subPath.size();
        return maxLen;
    }

    public Map<String, Double> getStatisticsExecution() {
        return statisticsExecution;
    }

    public Map<String, Integer> getStatisticsMission() {
        return statisticsMission;
    }

    public String getStatistics() {

        StringBuilder statistics = new StringBuilder();

        statistics.append("Statistics").append("\n");

        statistics.append("Mission statistics:").append("\n");
        for (Map.Entry<String, Integer> entry : (new TreeMap<>(statisticsMission)).entrySet()) {
            String k = entry.getKey();
            Integer v = entry.getValue();
            statistics.append("    ").append(k.substring(6)).append(": ").append(v).append("\n");
        }
        statistics.append("\n");

        statistics.append("Execution statistics:").append("\n");

        for (Map.Entry<String, Double> entry : (new TreeMap<>(statisticsExecution)).entrySet()) {
            String k = entry.getKey();
            Double v = entry.getValue();
            statistics.append("    ").append(k.substring(6)).append(": ").append(String.format("%.6f", v)).append("\n");
        }

        System.out.println("\n# # #");
        System.out.println(statistics);
        System.out.println("# # #");
        return statistics.toString();

    }

    public List<List<Integer>> getNumCellsPerEachUavSubPath() {
        return numCellsPerEachUavSubPath;
    }
}
