class mailer:
    username = ""
    password = ""
    to = ""

    def __init__(self, usr, pas, tomail):
        self.password = pas
        self.to = tomail
        self.username = usr

    def sendmail(self, msg1, subject=""):
        import datetime
        import smtplib
        server = smtplib.SMTP('smtp.gmail.com', 587)
        server.starttls()
        server.login(user=self.username, password=self.password)
        msgsubject = "Notification - HongLab [" + str(subject) + " ]"
        msg = "**** Notification - HongLab *****\nNotification Sent at : "
        msg += str(datetime.datetime.now()) + "\n"
        msg += msg1
        message = 'Subject: {}\n\n{}'.format(msgsubject, msg)
        print("Sending Notification")
        # server.sendmail(self.username, self.to, message)
        print("Notification Sent")
        server.quit()
