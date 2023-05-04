import numpy as np
import pandas
import matplotlib.pyplot as plt

if __name__ == "__main__":
    COLUMNS = {
        "_": -1,
        "POP": 0,
        "SEED": 1,
        "TOTAL_STEPS": 2,
        'CONVEX_HULL': 3,
        "INVERSE_HULL": 4,
        "ELEMS_0D": 5,
        'ELEMS_1D': 6,
        "MAX_0D": 7,
        "MAX_1D": 8,
    }

    # Import Data
    cyclic_ds = pandas.read_csv("out/geometry-cyclic.csv")
    cyclic_ds.columns = COLUMNS

    aggregation_ds = pandas.read_csv("out/geometry-aggregation.csv")
    aggregation_ds.columns = COLUMNS

    dispersal_ds = pandas.read_csv("out/geometry-dispersal.csv")
    dispersal_ds.columns = COLUMNS

    milling_ds = pandas.read_csv("out/geometry-milling.csv")
    milling_ds.columns = COLUMNS

    wall_f_ds = pandas.read_csv("out/geometry-wall-following.csv")
    wall_f_ds.columns = COLUMNS

    random_ds = pandas.read_csv("out/geometry-random.csv")
    random_ds.columns = COLUMNS

    def convex_hull_ratio():
        plt.figure(figsize=(8, 4))
        total = 0
        x, y = cyclic_ds["CONVEX_HULL"], cyclic_ds["INVERSE_HULL"]
        y = np.divide(cyclic_ds["INVERSE_HULL"], cyclic_ds["CONVEX_HULL"])
        plt.scatter(x, y, c='orange', label="Cyclic Pursuit")
        total += len(x)

        x, y = aggregation_ds["CONVEX_HULL"], aggregation_ds["INVERSE_HULL"]
        y = np.divide(aggregation_ds["INVERSE_HULL"], aggregation_ds["CONVEX_HULL"])
        plt.scatter(x, y, c='blue', label="Aggregation")
        total += len(x)

        x, y = dispersal_ds["CONVEX_HULL"], dispersal_ds["INVERSE_HULL"]
        y = np.divide(dispersal_ds["INVERSE_HULL"], dispersal_ds["CONVEX_HULL"])
        plt.scatter(x, y, c='green', label="Dispersal")
        total += len(x)

        x, y = milling_ds["CONVEX_HULL"], milling_ds["INVERSE_HULL"]
        y = np.divide(milling_ds["INVERSE_HULL"], milling_ds["CONVEX_HULL"])
        plt.scatter(x, y, c='red', label="Milling")
        total += len(x)

        x, y = wall_f_ds["CONVEX_HULL"], wall_f_ds["INVERSE_HULL"]
        y = np.divide(wall_f_ds["INVERSE_HULL"], wall_f_ds["CONVEX_HULL"])
        plt.scatter(x, y, c='purple', label="Wall Following")
        total += len(x)

        plt.xlabel("Convex Hull Area")
        plt.ylabel("Hull:Frame Ratio")
        plt.title("Swarm Behavior Space")
        plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
        plt.tight_layout()
        plt.show()
        print(f"TOTAL: {total}")

    def persistence_3d():
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        total = 0

        DIVIDE_X_BY_Y = False
        X_FIELD, Y_FIELD, Z_FIELD = "CONVEX_HULL", "INVERSE_HULL", "MAX_1D"

        x, y, z = cyclic_ds[X_FIELD], cyclic_ds[Y_FIELD], cyclic_ds[Z_FIELD]
        y = np.divide(cyclic_ds[Y_FIELD], cyclic_ds[X_FIELD]) if DIVIDE_X_BY_Y else y
        ax.scatter(x, y, z, c='orange', label="Cyclic Pursuit")


        x, y, z = aggregation_ds[X_FIELD], aggregation_ds[Y_FIELD], aggregation_ds[Z_FIELD]
        y = np.divide(aggregation_ds[Y_FIELD], aggregation_ds[X_FIELD]) if DIVIDE_X_BY_Y else y
        ax.scatter(x, y, z, c='blue', label="Aggregation")

        x, y, z = dispersal_ds[X_FIELD], dispersal_ds[Y_FIELD], dispersal_ds[Z_FIELD]
        y = np.divide(dispersal_ds[Y_FIELD], dispersal_ds[X_FIELD]) if DIVIDE_X_BY_Y else y
        ax.scatter(x, y, z, c='green', label="Dispersal")

        x, y, z = milling_ds[X_FIELD], milling_ds[Y_FIELD], milling_ds[Z_FIELD]
        y = np.divide(milling_ds[Y_FIELD], milling_ds[X_FIELD]) if DIVIDE_X_BY_Y else y
        ax.scatter(x, y, z, c='red', label="Milling")

        x, y, z = wall_f_ds[X_FIELD], wall_f_ds[Y_FIELD], wall_f_ds[Z_FIELD]
        y = np.divide(wall_f_ds[Y_FIELD], wall_f_ds[X_FIELD]) if DIVIDE_X_BY_Y else y
        ax.scatter(x, y, z, c='purple', label="Wall Following")

        total_size = len(wall_f_ds) + len(milling_ds) + len(dispersal_ds) + len(aggregation_ds) + len(cyclic_ds)
        print(f"Total Controllers: {total_size}")

        plt.xlabel("Hull Area")
        plt.ylabel("Internal Frame Area")
        ax.set_zlabel("Max H1 Death Time")
        plt.title("Swarm Behavior Space")
        plt.legend()
        plt.show()


    convex_hull_ratio()