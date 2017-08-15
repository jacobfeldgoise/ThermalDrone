import time, tzlocal, pytz
from datetime import datetime                                                   # Needed to generate timestamp
from dateutil import tz                                                         # Needed to generate timestamp


def timestamp_img():

    #### This function generates a timestemp which will be written to the log file ####

    local_timezone = tzlocal.get_localzone()
    utc_time = datetime.utcnow()

    # Convert time zone
    local_time= utc_time.replace(tzinfo=pytz.utc).astimezone(local_timezone)
    timestamp_local = local_time.strftime('%m-%d-%Y-%H-%M-%S')
    return timestamp_local

print "img-" + str(timestamp_img()) + ".jpg"
