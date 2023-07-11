import numpy

# Create a simulation of a bank account.


# The account should have a balance, a name and an account number.
# The account should have a method to withdraw money.
# The account should have a method to deposit money.
# The account should have a method to print the current balance.
class BankAcct:
    def __init__(self, balance, name, acctNum):
        self.balance = balance
        self.name = name
        self.acctNum = acctNum

    def withdraw(self, amt):
        self.balance -= amt
        return amt

    def deposit(self, amt):
        self.balance += amt
        return amt

    def printBalance(self):
        print("Current balance: " + str(self.balance))
        return "Current balance: " + str(self.balance)

    def getBalance(self):
        return self.balance
