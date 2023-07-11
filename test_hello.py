import unittest
import hello


class TestHello(unittest.TestCase):
    def test_hello(self):
        self.assertEqual(hello.hello(), "Hello, world!")

    def test_add(self):
        self.assertEqual(hello.add(0, 0), 0)
        self.assertEqual(hello.add(1, 1), 2)
        self.assertNotEqual(hello.add(1, 1), 5)

    def test_sub(self):
        self.assertEqual(hello.sub(0, 0), 0)
        self.assertEqual(hello.sub(1, 0), 1)
        self.assertNotEqual(hello.sub(1, 2), 1)

    def test_mul(self):
        self.assertEqual(hello.mul(0, 0), 0)
        self.assertEqual(hello.mul(1, 1), 1)
        self.assertNotEqual(hello.mul(2, 2), 16)

    def test_div(self):
        self.assertEqual(hello.div(0, 1), 0)
        self.assertEqual(hello.div(5, 2), 2.5)
        self.assertNotEqual(hello.div(1, 1), 15)

    def test_sqrt(self):
        self.assertEqual(hello.sqrt(4), 2)
        self.assertEqual(hello.sqrt(9), 3)
        self.assertNotEqual(hello.sqrt(16), 2)

    def test_power(self):
        self.assertEqual(hello.power(1, 2), 1)
        self.assertEqual(hello.power(2, 2), 4)
        self.assertNotEqual(hello.power(3, 3), 15)

    def test_log(self):
        self.assertEqual(hello.log(10), 2.302585092994046)
        self.assertEqual(hello.log(2), 0.6931471805599453)
        self.assertNotEqual(hello.log(5), 1.609437912434103)

    def test_exp(self):
        self.assertEqual(hello.exp(1), 2.718281828459045)
        self.assertEqual(hello.exp(2), 7.38905609893065)
        self.assertNotEqual(hello.exp(3), 20.08553692187668)

    def test_sin(self):
        self.assertEqual(hello.sin(0), 0)
        self.assertEqual(hello.sin(1), 0.8414709848078965)
        self.assertNotEqual(hello.sin(90), 0.893996663005579)

    def test_cos(self):
        self.assertEqual(hello.cos(0), 1)
        self.assertEqual(hello.cos(1), 0.5403023058681398)
        self.assertNotEqual(hello.cos(90), 0.4480736161291701)

    def test_tan(self):
        self.assertEqual(hello.tan(0), 0)
        self.assertEqual(hello.tan(1), 1.5574077246549023)
        self.assertNotEqual(hello.tan(90), 1.995200412208242)

    def test_cot(self):
        self.assertEqual(hello.cot(0), float("inf"))
        self.assertEqual(hello.cot(1), 0.6420926159343306)
        self.assertNotEqual(hello.cot(90), 0.5012027833801532)


if __name__ == "__main__":
    unittest.main()
