public class Score {

    public static int getIndexOfMaxValue(double[] a) {
        double maxValue = Integer.MIN_VALUE;
        int maxValueIndex = 0;

        for (int i = 0; i < a.length; i++) {
            if (a[i] > maxValue) {
                maxValue = a[i];
                maxValueIndex = i;
            }
        }
        return maxValueIndex;
    }

    public static VersusType getRoadsType(int[][] m) {

        int ROAD = 0;

        int size = m.length;
        int lastIndex = size - 1;

        ArrayList<Integer> horizontal_roads = new ArrayList<>();
        ArrayList<Integer> vertical_roads = new ArrayList<>();

        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {

                if (m[i][j] == ROAD) {

                    if (j < lastIndex && m[i][j + 1] == ROAD) {

                        int roadLen = 1;

                        while (j < lastIndex && m[i][j + 1] == ROAD) {
                            j++;
                            roadLen++;
                        }

                        horizontal_roads.add(roadLen);
                    }
                }
            }
        }

        for (int j = 0; j < size; j++) {
            for (int i = 0; i < size; i++) {
                if (m[i][j] == ROAD) {
                    if (i < lastIndex && m[i + 1][j] == ROAD) {
                        int roadLen = 1;
                        while (i < lastIndex && m[i + 1][j] == ROAD) {
                            i++;
                            roadLen++;
                        }
                        vertical_roads.add(roadLen);
                    }
                }
            }
        }

        int numHorizontalRoads = horizontal_roads.size();
        int totLenHorizontalRoads = horizontal_roads.stream().mapToInt(a -> a).sum();
        double avgLenHorizontalRoads = horizontal_roads.stream().mapToInt(a -> a).average().orElse(0);

        int numVerticalRoads = vertical_roads.size();
        int totLenVerticalRoads = vertical_roads.stream().mapToInt(a -> a).sum();
        double avgLenVerticalRoads = vertical_roads.stream().mapToInt(a -> a).average().orElse(0);

        if (avgLenHorizontalRoads > avgLenVerticalRoads)
            return VersusType.HORIZONTAL;
        else
            return VersusType.VERTICAL;
    }

    public static HashMap<VersusType, Double> getPathsType(int[][] m) {

        int size = m.length;
        int lastIndex = size - 1;

        ArrayList<Integer> horizontal_roads = new ArrayList<>();
        ArrayList<Integer> vertical_roads = new ArrayList<>();

        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                if (m[i][j] == PathDirection.LEFT || m[i][j] == PathDirection.RIGHT) {
                    if (j < lastIndex && (m[i][j] == PathDirection.LEFT || m[i][j] == PathDirection.RIGHT)) {
                        int roadLen = 1;
                        while (j < lastIndex && (m[i][j] == PathDirection.LEFT || m[i][j] == PathDirection.RIGHT)) {
                            j++;
                            roadLen++;
                        }
                        horizontal_roads.add(roadLen);
                    }
                }
            }
        }

        for (int j = 0; j < size; j++) {
            for (int i = 0; i < size; i++) {
                if (m[i][j] == PathDirection.UP || m[i][j] == PathDirection.DOWN) {
                    if (i < lastIndex && (m[i][j] == PathDirection.UP || m[i][j] == PathDirection.DOWN)) {
                        int roadLen = 1;
                        while (i < lastIndex && (m[i][j] == PathDirection.UP || m[i][j] == PathDirection.DOWN)) {
                            i++;
                            roadLen++;
                        }
                        vertical_roads.add(roadLen);
                    }
                }
            }
        }

        int numHorizontalRoads = horizontal_roads.size();
        int totLenHorizontalRoads = horizontal_roads.stream().mapToInt(a -> a).sum();
        double avgLenHorizontalRoads = horizontal_roads.stream().mapToInt(a -> a).average().orElse(0);

        int numVerticalRoads = vertical_roads.size();
        int totLenVerticalRoads = vertical_roads.stream().mapToInt(a -> a).sum();
        double avgLenVerticalRoads = vertical_roads.stream().mapToInt(a -> a).average().orElse(0);

        HashMap<VersusType, Double> directionAndScore = new HashMap<>();

        if (avgLenHorizontalRoads > avgLenVerticalRoads)

            directionAndScore.put(VersusType.HORIZONTAL, (double) numVerticalRoads);
        else
            directionAndScore.put(VersusType.VERTICAL, (double) numHorizontalRoads);

        return directionAndScore;
    }

    public static double calculateScoreMST(
            int[][] region,
            int[][] pathSTC,
            int modePathSTC) {
        double myScore = 0;

        VersusType regionVersus = getRoadsType(region);

        HashMap<VersusType, Double> directionAndScore = getPathsType(pathSTC);
        Map.Entry<VersusType, Double> entry = directionAndScore.entrySet().iterator().next();

        VersusType pathVersus = entry.getKey();
        myScore += entry.getValue();

        if (regionVersus != pathVersus) {
            myScore += region.length * region.length;
        }

        myScore += getBorderOverlapScore(region, pathSTC, modePathSTC);
        return myScore;
    }

    private static double getBorderOverlapScore(int[][] region, int[][] pathSTC, int stcMode) {
        double overlapScore = 0;

        assert region.length == region[0].length;

        int regionSize = region.length;
        int lastIndex = regionSize - 1;

        int ROAD = 0;
        int NO_PATH = 0;
        ArrayList<Integer> overlapRoads = new ArrayList<>();

        if (stcMode == ModePathSTC.HORIZONTAL.UP || stcMode == ModePathSTC.HORIZONTAL.DOWN) {
            int row;

            if (stcMode == ModePathSTC.HORIZONTAL.UP) {
                int firstPathRow = 0;
                for (int i = 0; i < regionSize; i++) {
                    for (int j = 0; j < regionSize; j++) {
                        if (pathSTC[i][j] != NO_PATH) {
                            firstPathRow = i;
                            i = regionSize;
                            break;
                        }
                    }
                }
                row = firstPathRow;
            } else {
                int lastRowPath = 0;
                for (int i = lastIndex; i >= 0; i--) {
                    for (int j = 0; j < regionSize; j++) {
                        if (pathSTC[i][j] != NO_PATH) {
                            lastRowPath = i;
                            i = -1;
                            break;
                        }
                    }
                }
                row = lastRowPath;

            }

            for (int col = 0; col < regionSize; col++) {
                if (region[row][col] == ROAD) {
                    int roadLen = 1;
                    while (col < lastIndex && region[row][col + 1] == ROAD) {
                        col++;
                        roadLen++;
                    }
                    overlapRoads.add(roadLen);
                }
            }

        } else if (stcMode == ModePathSTC.VERTICAL.LEFT || stcMode == ModePathSTC.VERTICAL.RIGHT) {
            int col;

            if (stcMode == ModePathSTC.VERTICAL.LEFT) {
                int firstPathCol = 0;
                for (int j = 0; j < regionSize; j++) {
                    for (int i = 0; i < regionSize; i++) {
                        if (pathSTC[i][j] != NO_PATH) {
                            firstPathCol = i;
                            j = regionSize;
                            break;
                        }
                    }
                }
                col = firstPathCol;
            } else {
                int lastPathCol = 0;
                for (int j = lastIndex; j >= 0; j--) {
                    for (int i = 0; i < regionSize; i++) {
                        if (pathSTC[j][i] != NO_PATH) {
                            lastPathCol = j;
                            j = -1;
                            break;
                        }
                    }
                }
                col = lastPathCol;

            }

            for (int row = 0; row < lastIndex; row++) {
                if (region[row][col] == ROAD) {
                    int roadLen = 1;
                    while (row < lastIndex && region[row + 1][col] == ROAD) {
                        row++;
                        roadLen++;
                    }
                    overlapRoads.add(roadLen);
                }
            }

        }

        double size = overlapRoads.size();
        double sum = overlapRoads.stream().mapToInt(e -> e).sum() / (double) regionSize;

        overlapScore += size + sum;

        return overlapScore;
    }

    public enum VersusType {
        HORIZONTAL, VERTICAL
    }

    public static class ModePathSTC {
        public static class HORIZONTAL {
            public static final int DOWN = 0;
            public static final int UP = 1;

        }

        public static class VERTICAL {
            public static final int RIGHT = 2;
            public static final int LEFT = 3;

        }
    }

    public static class PathDirection {
        public static int UP = 1;
        public static int LEFT = 2;
        public static int RIGHT = 3;
        public static int DOWN = 4;
    }

}
