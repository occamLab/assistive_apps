import json
import gmplot


def main():
    debugging = False

    # FIXME: this path requires that you run python gpsvioplot.py from the containing folder only
    f = open('../data/the_o_ccw.json', 'r')
    obj = json.load(f)
    f.close()

    latitudes, longitudes = zip(*obj["gps_crumbs"])
    print "Length of gps markers: ", len(latitudes)

    if debugging:
        new_lat, new_lon = [], []
        for i, (lat, lon) in enumerate(zip(latitudes, longitudes)):
            new_lat.append(lat - 0.0001 * i)
        latitudes = new_lat

    gmap = gmplot.GoogleMapPlotter(latitudes[0], longitudes[0], 18)

    gmap.scatter(latitudes, longitudes, 'g', marker=True)

    gmap.draw("mymap.html")


if __name__ == "__main__":
    main()
