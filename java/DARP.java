/**
 * Originally created by atkap on 5/16/2016.
 * Adapted by Lorenzo Porcelli.
 * [Refercence]
 * Kapoutsis, Athanasios Ch, Savvas A. Chatzichristofis, and Elias B. Kosmatopoulos.
 * "DARP: divide areas algorithm for optimal multi-robot coverage path planning."
 * Journal of Intelligent & Robotic Systems 86.3-4 (2017): 663-680.
 */
public class DARP {
    private final double variateWeight;
    private final double randomLevel;
    private final int rows;
    private final int cols;
    private final int[][] GridEnv;
    private final ArrayList<Integer[]> droneInitialPosition;
    private final int[][] A;
    private final boolean[][] robotBinary;
    private final int discr;
    private final boolean UseImportance;

    private ArrayList<int[][]> binaryDroneCellAssignmentList;
    private boolean canceled;
    private int numOfDrones;
    private int[] cellPerDroneCounter;
    private boolean[] ConnectedRobotRegions;
    private boolean success;
    private ArrayList<boolean[][]> BinaryRobotRegions;
    private int maxCellsAss, minCellsAss;
    private double elapsedTime;
    private int numOfObstacles;
    private int maxIter;
    private int totalIterations;
    private final double[] droneAreaPortion;
    private double[] desiderableAssign;

    public DARP(int r, int c, int[][] src,
                int iters, double vWeight, double rLevel, int discr, boolean imp, double[] droneAreaPortion
    ) {
        this.rows = r;
        this.cols = c;
        this.GridEnv = deepCopyMatrix(src); // -1 cella vuota, -2 ostacolo, i drone (con i = 0,1,2,...)
        this.numOfDrones = 0;
        this.numOfObstacles = 0;
        this.maxIter = iters;
        this.droneInitialPosition = new ArrayList<>();
        this.A = new int[rows][cols];
        this.robotBinary = new boolean[rows][cols]; // posizione booleana droni

        this.variateWeight = vWeight;
        this.randomLevel = rLevel;
        this.discr = discr;
        this.canceled = false;
        this.UseImportance = imp;
        this.droneAreaPortion = droneAreaPortion;
        defineRobotsObstacles();
    }

    public void constructAssignmentM() {

        long startTime = System.nanoTime();

        //Constant Initializations
        int totalCells = rows * cols;
        int regionEffectiveSize = totalCells - numOfDrones - numOfObstacles;

        int termThreshold = 0;
        if (regionEffectiveSize % numOfDrones != 0) {
            termThreshold = 1;
        }

        desiderableAssign = new double[numOfDrones];
        for (int i = 0; i < numOfDrones; i++) {
            desiderableAssign[i] = regionEffectiveSize * droneAreaPortion[i];
            if (desiderableAssign[i] != (int) desiderableAssign[i] && termThreshold != 1) {
                termThreshold = 1;
            }
        }

        ArrayList<double[][]> matrixDistancesList = new ArrayList<>();
        ArrayList<double[][]> evaluationMatrixList = new ArrayList<>();

        for (int d = 0; d < numOfDrones; d++) {
            matrixDistancesList.add(new double[rows][cols]);
            evaluationMatrixList.add(new double[rows][cols]);
        }

        double[] maximumDist, maximumImportance, minimumImportance;

        maximumDist = new double[numOfDrones];
        maximumImportance = new double[numOfDrones];
        minimumImportance = new double[numOfDrones];
        for (int d = 0; d < numOfDrones; d++) {
            minimumImportance[d] = Double.MAX_VALUE;
        }

        float[][] ONES2D = new float[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {

                double tempSum = 0.0;

                for (int d = 0; d < numOfDrones; d++) {
                    matrixDistancesList.get(d)[i][j] = EuclideanDis(droneInitialPosition.get(d), new Integer[]{i, j});
                    if (matrixDistancesList.get(d)[i][j] > maximumDist[d]) {
                        maximumDist[d] = matrixDistancesList.get(d)[i][j];
                    }
                    tempSum += matrixDistancesList.get(d)[i][j];
                }

                for (int d = 0; d < numOfDrones; d++) {
                    evaluationMatrixList.get(d)[i][j] = 1.0 / (tempSum - matrixDistancesList.get(d)[i][j]);

                    if (evaluationMatrixList.get(d)[i][j] > maximumImportance[d]) {
                        maximumImportance[d] = evaluationMatrixList.get(d)[i][j];
                    }

                    if (evaluationMatrixList.get(d)[i][j] < minimumImportance[d]) {
                        minimumImportance[d] = evaluationMatrixList.get(d)[i][j];
                    }
                }
                ONES2D[i][j] = 1;
            }
        }

        success = false;


        ArrayList<double[][]> metricMatrixList = deepCopyListMatrix(matrixDistancesList);

        double[][] criterionMatrix = new double[rows][cols];

        while (termThreshold <= discr && !success && !canceled) {

            // Initializations
            double downThreshold = ((double) totalCells - (double) termThreshold * (numOfDrones - 1)) / (double) (totalCells * numOfDrones);
            double upperThreshold = ((double) totalCells + termThreshold) / (double) (totalCells * numOfDrones);

            success = true;

            // Main optimization loop
            int iter = 0;
            while (iter <= maxIter && !canceled) {

                assign(metricMatrixList);

                ArrayList<float[][]> ConnectedMultiplierList = new ArrayList<>();
                double[] plainErrors = new double[numOfDrones];
                double[] divFairError = new double[numOfDrones];

                // Connected Areas Component
                for (int d = 0; d < numOfDrones; d++) {

                    float[][] ConnectedMultiplier = deepCopyMatrix(ONES2D);
                    ConnectedRobotRegions[d] = true;

                    ConnectComponent cc = new ConnectComponent();
                    int[][] Ilabel = cc.compactLabeling(binaryDroneCellAssignmentList.get(d), new Dimension(cols, rows), true);
                    if (cc.getMaxLabel() > 1) { //At least one unconnected regions among d-robot's regions is found
                        ConnectedRobotRegions[d] = false;

                        //Find robot's sub-region and construct Robot and Non-Robot binary regions
                        cc.constructBinaryImages(Ilabel[droneInitialPosition.get(d)[0]][droneInitialPosition.get(d)[1]]);

                        //Construct the final Connected Component Multiplier
                        ConnectedMultiplier = CalcConnectedMultiplier(cc.NormalizedEuclideanDistanceBinary(true),
                                cc.NormalizedEuclideanDistanceBinary(false));

                    }
                    ConnectedMultiplierList.add(d, ConnectedMultiplier);

                    //Calculate the deviation from the Optimal Assignment
                    plainErrors[d] = cellPerDroneCounter[d] / (desiderableAssign[d] * numOfDrones);
                    //System.out.print(ArrayOfElements[d]+ ", ");
                    //divFairError[d] = fairDivision - plainErrors[d];

                    if (plainErrors[d] < downThreshold) {
                        divFairError[d] = downThreshold - plainErrors[d];
                    } else if (plainErrors[d] > upperThreshold) {
                        divFairError[d] = upperThreshold - plainErrors[d];
                    }
                }

                //System.out.print("Iteration: "+iter+", ");

                // Exit conditions
                if (isThisAGoalState(termThreshold)) {
                    break;
                }

                double TotalNegPerc = 0.0, totalNegPlainErrors = 0.0;
                double[] correctionMult = new double[numOfDrones];

                for (int r = 0; r < numOfDrones; r++) {
                    if (divFairError[r] < 0) {
                        TotalNegPerc += Math.abs(divFairError[r]);
                        totalNegPlainErrors += plainErrors[r];
                    }
                    correctionMult[r] = 1.0;
                }

                //Restore Fairness among the different partitions
                for (int r = 0; r < numOfDrones; r++) {
                    if (totalNegPlainErrors != 0.0) {

                        final double v = (plainErrors[r] / totalNegPlainErrors) * (TotalNegPerc / 2.0);
                        if (divFairError[r] < 0.0) {
                            correctionMult[r] = 1.0 + v;
                        } else {
                            correctionMult[r] = 1.0 - v;
                        }


                        criterionMatrix = calculateCriterionMatrix(evaluationMatrixList.get(r), minimumImportance[r], maximumImportance[r], correctionMult[r], (divFairError[r] < 0));
                    }
                    metricMatrixList.set(r, FinalUpdateOnMetricMatrix(criterionMatrix, generateRandomMatrix(), metricMatrixList.get(r), ConnectedMultiplierList.get(r)));
                }

                iter++;
            }

            this.totalIterations += iter;

            if (iter >= maxIter) {
                maxIter = maxIter / 2;
                success = false;
                termThreshold++;
            }
        }

        elapsedTime = (double) (System.nanoTime() - startTime) / Math.pow(10, 9);
        calculateRobotBinaryArrays();
    }

    private void calculateRobotBinaryArrays() {
        BinaryRobotRegions = new ArrayList<>();
        for (int r = 0; r < numOfDrones; r++) {
            BinaryRobotRegions.add(new boolean[rows][cols]);
        }
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (A[i][j] < numOfDrones) {
                    BinaryRobotRegions.get(A[i][j])[i][j] = true;
                }
            }
        }
    }

    // criterionMatrix, generateRandomMatrix(), MetricMatrix.get(r), ConnectedMultiplierList.get(r))
    private double[][] FinalUpdateOnMetricMatrix(double[][] criterionMatrix,
                                                 double[][] randomMatrix,
                                                 double[][] currentMetricMatrix,
                                                 float[][] connectedComponentMultiplier) {
        double[][] newMetricMatrix = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                newMetricMatrix[i][j] = currentMetricMatrix[i][j] * criterionMatrix[i][j] * randomMatrix[i][j] * connectedComponentMultiplier[i][j];
            }
        }

        return newMetricMatrix;
    }

    private double[][] generateRandomMatrix() {

        double[][] RandomMa = new double[rows][cols];
        Random randomno = new Random();

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                RandomMa[i][j] = 2.0 * randomLevel * randomno.nextDouble() + 1.0 - randomLevel;
            }
        }

        return RandomMa;
    }

    private double[][] calculateCriterionMatrix(double[][] TilesImp, double minImp, double maxImp, double correctiveMultiplier, boolean SmallerThan0) {
        double[][] retrunCriter = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (UseImportance) {
                    if (SmallerThan0) {
                        retrunCriter[i][j] = (TilesImp[i][j] - minImp) * ((correctiveMultiplier - 1) / (maxImp - minImp)) + 1;
                    } else {
                        retrunCriter[i][j] = (TilesImp[i][j] - minImp) * ((1 - correctiveMultiplier) / (maxImp - minImp)) + correctiveMultiplier;
                    }
                } else {
                    retrunCriter[i][j] = correctiveMultiplier;
                }
            }
        }

        return retrunCriter;
    }

    private boolean isThisAGoalState(int threshold) {
//        maxCellsAss = 0;
//        minCellsAss = Integer.MAX_VALUE;
//
//        for (int d = 0; d < numOfDrones; d++) {
//
//            if (maxCellsAss < cellPerDroneCounter[d]) {
//                maxCellsAss = cellPerDroneCounter[d];
//            }
//
//            if (minCellsAss > cellPerDroneCounter[d]) {
//                minCellsAss = cellPerDroneCounter[d];
//            }
//
//            if (!ConnectedRobotRegions[d]) {
//                return false;
//            }
//        }
//
//        return (maxCellsAss - minCellsAss) <= threshold;

        for (int r = 0; r < numOfDrones; r++) {
            if (Math.abs(desiderableAssign[r] - cellPerDroneCounter[r]) > threshold || !ConnectedRobotRegions[r]) {
                return false;
            }
        }

//        System.out.println("\nFinal assigment");
//        for (int i = 0; i < nr; i++) {
//            System.out.println("R"+(i+1)+"--> desired: "+DesireableAssign[i]+" | Converged: "+ArrayOfElements[i]);
//        }
//        System.out.println();

        return true;

    }

    private float[][] CalcConnectedMultiplier(float[][] dist1, float[][] dist2) {
        float[][] returnM = new float[rows][cols];
        float MaxV = 0;
        float MinV = Float.MAX_VALUE;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                returnM[i][j] = dist1[i][j] - dist2[i][j];
                if (MaxV < returnM[i][j]) {
                    MaxV = returnM[i][j];
                }
                if (MinV > returnM[i][j]) {
                    MinV = returnM[i][j];
                }
            }
        }

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                returnM[i][j] = (returnM[i][j] - MinV) * ((2 * (float) variateWeight) / (MaxV - MinV)) + (1 - (float) variateWeight);
            }
        }

        return returnM;
    }

    private void assign(ArrayList<double[][]> Q) {

        binaryDroneCellAssignmentList = new ArrayList<>();
        for (int r = 0; r < numOfDrones; r++) {
            binaryDroneCellAssignmentList.add(new int[rows][cols]);
            binaryDroneCellAssignmentList.get(r)[droneInitialPosition.get(r)[0]][droneInitialPosition.get(r)[1]] = 1;
        }

        cellPerDroneCounter = new int[numOfDrones];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {

                // assegnamento della cella a un drone
                if (GridEnv[i][j] == -1) {

                    double minValue = Q.get(0)[i][j];
                    int droneIndex = 0;

                    // cerca il drone che dista meno dalla cella
                    for (int r = 1; r < numOfDrones; r++) {
                        if (Q.get(r)[i][j] < minValue) {
                            minValue = Q.get(r)[i][j];
                            droneIndex = r;
                        }
                    }
                    A[i][j] = droneIndex;
                    binaryDroneCellAssignmentList.get(droneIndex)[i][j] = 1;
                    cellPerDroneCounter[droneIndex]++;
                } else if (GridEnv[i][j] == -2) {
                    A[i][j] = numOfDrones;
                }
            }
        }

    }

    private void defineRobotsObstacles() {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (GridEnv[i][j] == 2) { // nella cella c'è un drone
                    robotBinary[i][j] = true;
                    droneInitialPosition.add(new Integer[]{i, j});
                    GridEnv[i][j] = numOfDrones;
                    A[i][j] = numOfDrones;
                    numOfDrones++;
                } else if (GridEnv[i][j] == 1) { // nella cella c'è un ostacolo
                    numOfObstacles++;
                    GridEnv[i][j] = -2;
                } else { // la cella è vuota
                    GridEnv[i][j] = -1;
                }
            }
        }

        ConnectedRobotRegions = new boolean[numOfDrones];
    }

    public boolean getSuccess() {
        return success;
    }

    public int[][] getAssignmentMatrix() {
        return A;
    }

    public boolean[][] getRobotBinary() {
        return robotBinary;
    }

    public ArrayList<boolean[][]> getBinaryRobotRegions() {
        return BinaryRobotRegions;
    }

    public ArrayList<Integer[]> getDroneInitialPosition() {
        return droneInitialPosition;
    }

    public int getEffectiveSize() {
        return 4 * (rows * cols - numOfObstacles);
    }

    public int getMaxCellsAss() {
        return 4 * (maxCellsAss + 1);
    }

    public int getMinCellsAss() {
        return 4 * (minCellsAss + 1);
    }

    public double getElapsedTime() {
        return elapsedTime;
    }

    public int getDiscr() {
        return discr;
    }

    public int getTotalIterations() {
        return totalIterations;
    }

    public int getAchievedDiscr() {
        return maxCellsAss - minCellsAss;
    }

    public void setCanceled(boolean canceled) {
        this.canceled = canceled;
    }

}
