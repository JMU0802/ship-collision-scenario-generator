import math

WGS84_semimajor_axis_meters = 6378137.0
mercator_k0 = 0.9996
WGSinvf = 298.257223563

DEGREE = math.pi/180.0
RADIAN = 180.0/math.pi
HALFPI = math.pi/2
PI = math.pi
ONEPI = PI
TWOPI = PI*2
SPI = PI
MERI_TOL = 1e-9
DTOL = 1e-12

sin = math.sin
asin = math.asin
cos = math.cos
acos = math.acos
atan = math.atan
atan2 = math.atan2
tan = math.tan
log = math.log
exp = math.exp
sqrt = math.sqrt
floor = math.floor


def toSM(lat, lon, lat0, lon0):
    xlon = lon
    if (lon * lon0 < 0.) and (abs(lon - lon0) > 180.):
        if lon < 0.:
            xlon += 360.
        else:
            xlon -= 360.

    z = WGS84_semimajor_axis_meters * mercator_k0

    x1 = (xlon - lon0) * DEGREE * z
    x = x1

    s = sin(lat * DEGREE)
    y3 = (.5 * log((1 + s) / (1 - s))) * z

    s0 = sin(lat0 * DEGREE)
    y30 = (.5 * log((1 + s0) / (1 - s0))) * z
    y4 = y3 - y30

    y = y4

    return x, y


def fromSM(x, y, lat0, lon0):
    z = WGS84_semimajor_axis_meters * mercator_k0
    s0 = sin(lat0 * DEGREE)
    y0 = (.5 * log((1 + s0) / (1 - s0))) * z

    lat3 = (2.0 * atan(exp((y0+y)/z)) - PI/2.) / DEGREE
    lat = lat3

    lon1 = lon0 + (x / (DEGREE * z))
    lon = lon1

    return lat, lon


def toSM_ECC(lat, lon, lat0, lon0):
    f = 1.0 / WGSinvf
    e2 = 2 * f - f * f
    e = sqrt(e2)

    z = WGS84_semimajor_axis_meters * mercator_k0

    x = (lon - lon0) * DEGREE * z

    s = sin(lat * DEGREE)
    # y3 = (.5 * log((1 + s) / (1 - s))) * z

    s0 = sin(lat0 * DEGREE)
    # y30 = (.5 * log((1 + s0) / (1 - s0))) * z
    # y4 = y3 - y30

    falsen = z*log(tan(PI/4 + lat0 * DEGREE / 2)*pow(
        (1. - e * s0)/(1. + e * s0), e/2.))
    test = z*log(tan(PI/4 + lat * DEGREE / 2)*pow(
        (1. - e * s)/(1. + e * s), e/2.))
    y = test - falsen

    return x, y


def fromSM_ECC(x, y, lat0, lon0):
    f = 1.0 / WGSinvf
    es = 2 * f - f * f
    e = sqrt(es)

    z = WGS84_semimajor_axis_meters * mercator_k0

    lon = lon0 + (x / (DEGREE * z))

    s0 = sin(lat0 * DEGREE)

    falsen = z*log(tan(PI/4 + lat0 * DEGREE / 2)*pow(
        (1. - e * s0)/(1. + e * s0), e/2.))
    t = exp((y + falsen) / (z))
    xi = (PI / 2.) - 2.0 * atan(t)

    esf = (es/2. + (5*es*es/24.) + (es*es*es/12.) + (
        13.0*es*es*es*es/360.)) * sin(2 * xi)
    esf += ((7.*es*es/48.) + (29.*es*es*es/240.) + (
        811.*es*es*es*es/11520.)) * sin(4. * xi)
    esf += ((7.*es*es*es/120.) + (81*es*es*es*es/1120.) + (
        4279.*es*es*es*es/161280.)) * sin(8. * xi)

    lat = -(xi + esf) / DEGREE
    return lat, lon


def adjlon(lon):
    if abs(lon) <= SPI:
        return lon
    lon += ONEPI
    lon -= TWOPI * floor(lon / TWOPI)
    lon -= ONEPI
    return lon


def ll_gc_ll(lat, lon, brg, dist):
    phi1 = lat * DEGREE
    lam1 = lon * DEGREE
    al12 = brg * DEGREE
    geod_S = dist * 1852.0

    ellipse = 1
    f = 1.0 / WGSinvf
    geod_a = WGS84_semimajor_axis_meters

    es = 2 * f - f * f
    onef = sqrt(1. - es)
    geod_f = 1 - onef
    # f2 = geod_f/2
    f4 = geod_f/4
    # f64 = geod_f*geod_f/64

    al12 = adjlon(al12)
    signS = 1 if abs(al12) > HALFPI else 0
    th1 = atan(onef * tan(phi1)) if ellipse != 0 else phi1
    costh1 = cos(th1)
    sinth1 = sin(th1)
    sina12 = sin(al12)
    merid = 1 if abs(sina12) < MERI_TOL else 0
    if merid == 1:
        sina12 = 0.
        cosa12 = 1. if abs(al12) < HALFPI else -1.
        M = 0.
    else:
        cosa12 = cos(al12)
        M = costh1 * sina12

    N = costh1 * cosa12
    if ellipse != 0:
        if merid != 0:
            c1 = 0.
            c2 = f4
            D = 1. - c2
            D *= D
            P = c2 / D
        else:
            c1 = geod_f * M
            c2 = f4 * (1. - M * M)
            D = (1. - c2)*(1. - c2 - c1 * M)
            P = (1. + .5 * c1 * M) * c2 / D
    if merid != 0:
        s1 = HALFPI - th1
    else:
        s1 = 0. if abs(M) >= 1. else acos(M)
        s1 = sinth1 / sin(s1)
        s1 = 0. if abs(s1) >= 1. else acos(s1)

    ss = 0.
    if ellipse != 0:
        d = geod_S / (D * geod_a)
        if signS != 0:
            d = -d
        u = 2. * (s1 - d)
        V = cos(u + d)
        sind = sin(d)
        X = c2 * c2 * (sind) * cos(d) * (2. * V * V - 1.)
        ds = d + X - 2. * P * V * (1. - 2. * P * cos(u)) * sind
        ss = s1 + s1 - ds
    else:
        ds = geod_S / geod_a
        if signS != 0:
            ds = - ds

    cosds = cos(ds)
    sinds = sin(ds)
    if signS != 0:
        sinds = - sinds

    al21 = N * cosds - sinth1 * sinds
    if merid != 0:
        phi2 = atan(tan(HALFPI + s1 - ds) / onef)
        if al21 > 0.:
            al21 = PI
            if signS != 0:
                de = PI
            else:
                phi2 = - phi2
                de = 0.
        else:
            al21 = 0.
            if signS != 0:
                phi2 = - phi2
                de = 0
            else:
                de = PI
    else:
        al21 = atan(M / al21)
        if al21 > 0:
            al21 += PI
        if al12 < 0.:
            al21 -= PI
        al21 = adjlon(al21)
        phi2 = atan(-(sinth1 * cosds + N * sinds) * sin(al21) /
                    (onef*M if ellipse != 0 else M))
        de = atan2(sinds * sina12, (costh1 * cosds - sinth1 * sinds * cosa12))
        if ellipse != 0:
            if signS != 0:
                de += c1 * ((1. - c2) * ds + c2 * sinds * cos(ss))
            else:
                de -= c1 * ((1. - c2) * ds - c2 * sinds * cos(ss))
    lam2 = adjlon(lam1 + de)

    dlat = phi2 / DEGREE
    dlon = lam2 / DEGREE
    return dlat, dlon


def DistGreatCircle(slat, slon, dlat, dlon):
    phi1 = slat * DEGREE
    lam1 = slon * DEGREE
    phi2 = dlat * DEGREE
    lam2 = dlon * DEGREE

    ellipse = 1
    f = 1.0 / WGSinvf
    geod_a = WGS84_semimajor_axis_meters

    es = 2 * f - f * f
    onef = sqrt(1. - es)
    geod_f = 1 - onef
    # f2 = geod_f/2
    f4 = geod_f/4
    f64 = geod_f*geod_f/64

    if ellipse != 0:
        th1 = atan(onef * tan(phi1))
        th2 = atan(onef * tan(phi2))
    else:
        th1 = phi1
        th2 = phi2

    thm = .5 * (th1 + th2)
    dthm = .5 * (th2 - th1)
    dlam = adjlon(lam2-lam1)
    dlamm = .5 * (dlam)

    if abs(dlam) < DTOL and abs(dthm) < DTOL:
        # al12 = 0.
        # al21 = 0.
        geod_S = 0.
        return 0.0

    sindlamm = sin(dlamm)
    costhm = cos(thm)
    sinthm = sin(thm)
    cosdthm = cos(dthm)
    sindthm = sin(dthm)

    L = sindthm * sindthm + (cosdthm * cosdthm - sinthm * sinthm) *\
        sindlamm * sindlamm
    cosd = 1 - L - L
    d = acos(cosd)
    if ellipse != 0:
        E = cosd + cosd
        sind = sin(d)
        Y = sinthm * cosdthm
        Y *= (Y + Y) / (1. - L)
        T = sindthm * costhm
        T *= (T + T) / L
        X = Y + T
        Y -= T
        T = d / sind
        D = 4. * T * T
        A = D * E
        B = D + D
        geod_S = geod_a * sind * (T - f4 * (T * X - Y) +
                                  f64 * (X * (A + (T - .5 * (A - E)) * X) -
                                         Y * (B + E * Y) + D * X * Y))
        # tandlammp = tan(.5 * (dlam - .25 * (Y + Y - E * (4. - X)) *
        #                       (f2 * T + f64 * (32. * T - (20. * T - A)
        #                                        * X - (B + 4.)*Y))*tan(dlam)))
    else:
        geod_S = geod_a * d
        # tandlammp = tan(dlamm)

    # u = atan2(sindthm, (tandlammp * costhm))
    # v = atan2(cosdthm, (tandlammp * sinthm))
    # al12 = adjlon(TWOPI + v - u)
    # al21 = adjlon(TWOPI - v - u)

    d5 = geod_S / 1852.0
    return d5


def DistanceBearingMercator(lat0, lon0, lat1, lon1):
    lon0x = lon0
    lon1x = lon1

    if (lon0x * lon1x) < 0.:
        if lon0x < 0.0:
            lon0x += 360.0
        else:
            lon1x += 360.0

        if abs(lon0x - lon1x) > 180.:
            if lon0x > lon1x:
                lon0x -= 360.0
            else:
                lon1x -= 360.0
        lon1x += 360.
        lon0x += 360.

    mlat0 = lat0 + 1e-9 if abs(lat1-lat0) < 1e-9 else lat0
    east, north = toSM_ECC(lat1, lon1x, mlat0, lon0x)
    C = atan2(east, north)
    if cos(C) != 0.0:
        dlat = (lat1 - mlat0) * 60.
        dist = (dlat/cos(C))
    else:
        dist = DistGreatCircle(lat0, lon0, lat1, lon1)

    east, north = toSM_ECC(lat1, lon1x, lat0, lon0x)

    C = atan2(east, north)
    brgt = 180. + (C * 180. / PI)
    if brgt < 0:
        brg = brgt + 360.
    elif brgt >= 360.:
        brg = brgt - 360.
    else:
        brg = brgt

    return dist, brg


# ll_gc_ll(24, 118, 90, 1)
# DistanceBearingMercator(14, -11, 12, 118)
