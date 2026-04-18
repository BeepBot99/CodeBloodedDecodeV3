plugins {
    id("dev.frozenmilk.teamcode") version "11.1.0-1.1.2"
    id("dev.frozenmilk.sinister.sloth.load") version "0.2.4"
}

ftc {
    sdk {
        TeamCode("11.1.0")
    }

    dairy {
        implementation(Sloth)
        implementation(slothboard)
        implementation(ftControl.fullpanels)
    }

    pedro {
        implementation(ftc("2.1.1"))
        implementation(telemetry)
    }
}

dependencies {
    implementation("com.github.haifengl:smile-interpolation:2.6.0") {
        exclude(group = "org.slf4j")
    }
    implementation("com.pedropathing:ivy:1.0.0")
    implementation("dev.frozenmilk.dairy:CachingHardware:1.0.0")
}