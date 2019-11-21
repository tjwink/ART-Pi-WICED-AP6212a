http://www.wavpack.com/

three version of the codec are located in this folder:

full :  the main library source code

tiny :  the light version (so called "tiny") for deeply embedded devices
        which does have ARM optimizations.
        note that the tiny version might be behind the full one.
        (in terms of supported features).

shortbloks: this is a special version of the codec that has been
            customized for RTP audio traffic where each payload
            is a small wavpack block.
