from datetime import timedelta

def get_h_m_s(td: timedelta):
    m, s = divmod(td.seconds, 60)
    h, m = divmod(m, 60)
    return h, m, s

