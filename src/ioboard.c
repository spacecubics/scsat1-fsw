/*
 * Copyright (c) 2023 Space Cubics, LLC.
 */

/* A weak version of the function. Each IO board should define its own
 * function.  Note that XC8 for PIC16LF877 with C90 library doesn't
 * have weak attribute.  This will be linked at very last to full
 * fill if not defined. */
int ioboard_init(void) {
        return 0;
};
