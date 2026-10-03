import logging
import time

from c2station import config
from c2station.link import Link

logging.basicConfig(level=logging.DEBUG, format="%(levelname)s %(name)s: %(message)s")

link = Link(config.CONNECTION_STRING)
link.connect()
link.start()

for _ in range(30):
    s = link.get_state()
    print(f"{s.mode:<8} armed={s.armed!s:<5} alt={s.alt}  lat={s.lat}  lon={s.lon}  "
          f"batt={s.battery_percent}%  sats={s.satellites}  wp={s.current_wp}")
    time.sleep(1)

print("Messages:", link.recent_messages()[-5:])