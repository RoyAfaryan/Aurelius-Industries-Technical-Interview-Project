import argparse
import logging
 
from c2station import config
from c2station.ui.app import App
 
 
def main() -> None:
    parser = argparse.ArgumentParser(description="C2 Station: a small ground station for ArduCopter")
    parser.add_argument("--conn", default=config.DEFAULT_CONNECTION_STRING,
                        help="MAVLink connection string, e.g. tcp:127.0.0.1:5762")
    parser.add_argument("--debug", action="store_true", help="verbose logging")
    args = parser.parse_args()
 
    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    App(default_conn=args.conn).mainloop()
 
 
if __name__ == "__main__":
    main()