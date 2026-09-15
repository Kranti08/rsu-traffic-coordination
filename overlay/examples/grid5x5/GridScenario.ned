import org.car2x.veins.nodes.RSU;
import org.car2x.veins.nodes.Scenario;

network GridScenario extends Scenario
{
    submodules:
        rsu[5]: RSU {
            @display("i=veins/sign/yellowdiamond;is=vs");
        }
}
