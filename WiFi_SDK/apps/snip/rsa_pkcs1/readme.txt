1. Generate 2048 bit RSA key
ssh-keygen -t rsa -b 2048
2. Dump RSA key components
openssl rsa -in <priv key file> -text
3. Generate 2048 bit Prime number
openssl prime -generate -bits 2048 -hex 