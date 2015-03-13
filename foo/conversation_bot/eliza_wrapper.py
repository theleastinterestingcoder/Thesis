import eliza

therapist = eliza.eliza()

while (True):
    sentence = raw_input("Your response:")
    print therapist.respond(sentence)

