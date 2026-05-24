"""Regression test: tf_transformations must import and round-trip on the current numpy.

The Jazzy container's apt-installed transforms3d 0.4.1 calls np.maximum_sctype, which
NumPy 2.0 removed. The container Dockerfile pip-installs transforms3d>=0.4.2 over the
apt one. If a future image rebuild loses that override (or a future numpy bump breaks
0.4.2 in turn), this test goes red instead of every chapter-3+ controller exploding
silently at runtime.
"""


def test_tf_transformations_imports_and_roundtrips():
    from tf_transformations import euler_from_quaternion, quaternion_from_euler

    roll, pitch, yaw = 0.1, 0.2, 0.3
    q = quaternion_from_euler(roll, pitch, yaw)
    r, p, y = euler_from_quaternion(q)
    assert abs(r - roll) < 1e-9
    assert abs(p - pitch) < 1e-9
    assert abs(y - yaw) < 1e-9
