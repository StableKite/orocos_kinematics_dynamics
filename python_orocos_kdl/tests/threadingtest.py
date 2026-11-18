import gc
import sys
import sysconfig
import unittest
import weakref
from concurrent.futures import ThreadPoolExecutor
from threading import Barrier

FREE_THREADED = bool(sysconfig.get_config_var("Py_GIL_DISABLED"))
GIL_BEFORE_IMPORT = sys._is_gil_enabled() if FREE_THREADED else True

import PyKDL as kdl


def make_chain():
    chain = kdl.Chain()
    chain.addSegment(kdl.Segment(kdl.Joint("x", kdl.Joint.TransX)))
    return chain


class ThreadingTest(unittest.TestCase):
    @unittest.skipUnless(FREE_THREADED, "requires a free-threaded interpreter")
    def test_import_keeps_gil_disabled(self):
        self.assertFalse(GIL_BEFORE_IMPORT)
        self.assertFalse(sys._is_gil_enabled())

    def test_solvers_keep_their_chain_alive(self):
        constructors = [
            kdl.ChainFkSolverPos_recursive,
            kdl.ChainFkSolverVel_recursive,
            kdl.ChainIkSolverPos_LMA,
            kdl.ChainIkSolverVel_pinv,
            kdl.ChainIkSolverVel_pinv_givens,
            kdl.ChainIkSolverVel_pinv_nso,
            kdl.ChainIkSolverVel_wdls,
            kdl.ChainJntToJacSolver,
            kdl.ChainJntToJacDotSolver,
            lambda chain: kdl.ChainIdSolver_RNE(chain, kdl.Vector.Zero()),
            lambda chain: kdl.ChainDynParam(chain, kdl.Vector.Zero()),
        ]
        for constructor in constructors:
            with self.subTest(constructor=constructor):
                chain = make_chain()
                chain_ref = weakref.ref(chain)
                solver = constructor(chain)
                del chain
                gc.collect()
                self.assertIsNotNone(chain_ref())
                del solver
                gc.collect()
                self.assertIsNone(chain_ref())

    def test_nr_keeps_wrapped_solvers_alive(self):
        for limited in (False, True):
            with self.subTest(limited=limited):
                chain = make_chain()
                forward = kdl.ChainFkSolverPos_recursive(chain)
                velocity = kdl.ChainIkSolverVel_pinv(chain)
                refs = [weakref.ref(obj) for obj in (chain, forward, velocity)]
                if limited:
                    lower, upper = kdl.JntArray(1), kdl.JntArray(1)
                    lower[0], upper[0] = -1, 1
                    inverse = kdl.ChainIkSolverPos_NR_JL(
                        chain, lower, upper, forward, velocity
                    )
                else:
                    inverse = kdl.ChainIkSolverPos_NR(
                        chain=chain, fksolver=forward, iksolver=velocity
                    )
                del chain, forward, velocity
                gc.collect()
                self.assertTrue(all(ref() is not None for ref in refs))

                result = kdl.JntArray(1)
                target = kdl.Frame(kdl.Vector(0.25, 0, 0))
                self.assertEqual(inverse.CartToJnt(kdl.JntArray(1), target, result), 0)
                self.assertAlmostEqual(result[0], 0.25)
                del inverse
                gc.collect()
                self.assertTrue(all(ref() is None for ref in refs))

    def test_parallel_solvers_share_read_only_chain(self):
        chain = make_chain()
        workers = 8
        start = Barrier(workers, timeout=30)

        def solve(index):
            start.wait()
            forward = kdl.ChainFkSolverPos_recursive(chain)
            velocity = kdl.ChainIkSolverVel_pinv(chain)
            inverse = kdl.ChainIkSolverPos_NR(chain, forward, velocity)
            jacobian = kdl.ChainJntToJacSolver(chain)
            dynamics = kdl.ChainDynParam(chain, kdl.Vector.Zero())
            joints, initial, result, gravity = (kdl.JntArray(1) for _ in range(4))
            frame, jac = kdl.Frame(), kdl.Jacobian(1)
            for iteration in range(512):
                position = (index * 512 + iteration + 1) / 8192.0
                joints[0] = position
                self.assertEqual(forward.JntToCart(joints, frame), 0)
                self.assertAlmostEqual(frame.p[0], position)
                self.assertEqual(inverse.CartToJnt(initial, frame, result), 0)
                self.assertAlmostEqual(result[0], position)
                self.assertEqual(jacobian.JntToJac(joints, jac), 0)
                self.assertAlmostEqual(jac[0, 0], 1.0)
                self.assertEqual(dynamics.JntToGravity(joints, gravity), 0)
                self.assertAlmostEqual(gravity[0], 0.0)
            if FREE_THREADED:
                self.assertFalse(sys._is_gil_enabled())

        with ThreadPoolExecutor(max_workers=workers) as executor:
            list(executor.map(solve, range(workers)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
