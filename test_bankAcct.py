import unittest
import bankAcct


class TestBankAcct(unittest.TestCase):
    def test_deposit(self):
        bankInstance = bankAcct.BankAcct(balance=0, name="Kevin", acctNum=17)
        self.assertEqual(bankInstance.deposit(45.00), 45.00)
        self.assertEqual(bankInstance.deposit(-15.00), -15.00)
        self.assertNotEqual(bankInstance.deposit(0), 100)

    def test_withdraw(self):
        bankInstance = bankAcct.BankAcct(balance=1738, name="Kevin", acctNum=17)
        self.assertEqual(bankInstance.withdraw(5.00), 5.00)
        self.assertEqual(bankInstance.withdraw(-5.00), -5.00)
        self.assertNotEqual(bankInstance.withdraw(100.00), 2.00)

    def test_printBalance(self):
        bankInstance = bankAcct.BankAcct(balance=0, name="Kevin", acctNum=17)
        self.assertEqual(bankInstance.printBalance(), "Current balance: 0")
        self.assertNotEqual(bankInstance.printBalance(), 2)


if __name__ == "__main__":
    unittest.main()
