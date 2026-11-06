"""Small real helper for Codex probes; invalid samples remain counted."""
import math
import statistics


def summarize(values):
    values = list(values)
    finite = sorted(float(v) for v in values if v is not None and math.isfinite(v))
    def percentile(q):
        if not finite:
            return None
        i = (len(finite)-1)*q
        lo, hi = math.floor(i), math.ceil(i)
        return finite[lo]+(finite[hi]-finite[lo])*(i-lo)
    return {'count': len(values), 'valid': len(finite), 'invalid': len(values)-len(finite),
            'p50': percentile(.5), 'p95': percentile(.95), 'p99': percentile(.99),
            'max': finite[-1] if finite else None,
            'peak_to_peak': finite[-1]-finite[0] if finite else None,
            'mean': statistics.mean(finite) if finite else None}
