"""Spike 3: run 1, 2 and 4 cars, each with all five sensors, one case after another.

Each case starts a fresh simulator session. Prints steps per second and every sensor's rate
per case. Writes multicar.json to --out.
"""

import sys

from kite_bridge import common

CAR_COUNTS = (1, 2, 4)


def run(args):
    model = args.model
    cases = []
    for n_cars in CAR_COUNTS:
        with common.session(args) as sim:
            model = model or common.resolve_model(sim, None)
            cars = sim.load_scenario(model, n_cars)
            common.start_traffic(sim, args.traffic)
            for car in cars:
                sim.set_ai_mode(car, "traffic")
            result = common.measure_sensors(sim, cars, args.seconds)
            for car in cars:
                sim.set_ai_mode(car, "disabled")
            print(f"\n{common.format_result(result)}")
            cases.append(result)
            common.write_json(args.out, "multicar.json", {"model": model, "cases": cases})


def main(argv=None):
    return common.guarded(run, common.parse_args(__doc__, argv))


if __name__ == "__main__":
    sys.exit(main())
