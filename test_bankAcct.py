import unittest
import bankAcct


class TestBankAcct(unittest.TestCase):
    bankAcct(0, "Kevin", 1234567890)

    def test_deposit(self):
        self.assertEqual(bankAcct.deposit(45.00), 45.00)
        self.assertEqual(bankAcct.deposit(-15.00), -15.00)
        self.assertNotEqual(bankAcct.deposit(0), 100)

    def test_withdraw(self):
        self.assertEqual(bankAcct.withdraw(5.00), 5.00)
        self.assertEqual(bankAcct.withdraw(-5.00), -5.00)
        self.assertNotEqual(bankAcct.withdraw(100.00), 2.00)

    def test_printBalance(self):
        self.assertEqual(
            bankAcct.printBalance(), "Current balance: " + str(BankAcct.balance)
        )
        self.assertNotEqual(bankAcct.printBalance(), 2)


if __name__ == "__main__":
    unittest.main()
