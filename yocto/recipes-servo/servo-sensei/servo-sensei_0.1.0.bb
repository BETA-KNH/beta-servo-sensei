SUMMARY = "ST bus servo driver and CLI tools"
DESCRIPTION = "C++ driver and utilities for controlling ST-series bus servos \
over a UART/RS-485 interface. Provides scan_servos, servo_cli, and \
servo_controller binaries."
HOMEPAGE = ""
LICENSE = "CLOSED"

# ---------------------------------------------------------------------------
# Source
# Adjust SRC_URI and SRCREV to your actual repository / branch.
# ---------------------------------------------------------------------------
SRC_URI = "git://your-repo/servo_sensei.git;protocol=ssh;branch=main"
SRCREV  = "${AUTOREV}"
PV      = "0.1.0+git${SRCPV}"

S = "${WORKDIR}/git"

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
inherit cmake

# Disable unit tests — Yocto builds are offline (no GitHub FetchContent).
# Hardware tests are excluded from install rules regardless.
EXTRA_OECMAKE = "-DBUILD_TESTING=OFF"

# No runtime library deps beyond libc / libstdc++ (already pulled in).
DEPENDS = ""

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
# CMakeLists.txt install() rules handle binaries and the sample env file.
# Here we additionally install a ready-to-use servo.env from the sample.
do_install:append() {
    # Create the config directory (install() may not have created it yet
    # if the sample file rename happened at a different prefix).
    install -d ${D}${sysconfdir}/servo_sensei

    # Install the sample as the live config file.
    # Operators should edit /etc/servo_sensei/servo.env on the target to
    # set SERVO_PORT and SERVO_BAUD for their board's UART.
    if [ -f ${D}${sysconfdir}/servo_sensei/servo.env.sample ]; then
        install -m 0644 \
            ${D}${sysconfdir}/servo_sensei/servo.env.sample \
            ${D}${sysconfdir}/servo_sensei/servo.env
    else
        install -m 0644 \
            ${S}/conf/servo.env \
            ${D}${sysconfdir}/servo_sensei/servo.env
    fi
}

# ---------------------------------------------------------------------------
# Packaging
# ---------------------------------------------------------------------------
FILES:${PN} = " \
    ${bindir}/scan_servos \
    ${bindir}/servo_cli \
    ${bindir}/servo_controller \
    ${sysconfdir}/servo_sensei/servo.env \
    ${sysconfdir}/servo_sensei/servo.env.sample \
"

# Mark the live config as a conffile so opkg/rpm will not overwrite it
# with an upgrade if the user has edited it.
CONFFILES:${PN} = "${sysconfdir}/servo_sensei/servo.env"
