#include <complex>

#include "fcontrol.h"
#include <math.h>

#include "IPlot.h"

using namespace std;

int main()
{



//    SystemBlock sys(vector<double>{1,1,1},vector<double>{0.5,-0.9,1});
    SystemBlock sys(vector<double>{1},vector<double>{-199,201});

    int numOrder=sys.GetNumOrder(),denOrder=sys.GetDenOrder();
    OnlineSystemIdentification Gz(numOrder,denOrder);

    IPlot real,id;
    double dts=0.01;
    double in=0,out=0;
    double tmax=20;

    for (double t=0; t<tmax; t+=dts)

    {
        in=1+0.01*((rand() % 10 + 1)-5); //u_{i-1}
        out=in > sys;//y_{i}

        Gz.UpdateSystemPEff(in,out);

        real.pushBack(out);
        //Gz.PrintParamsVector();

    }
    real.Plot();

    Gz.PrintZTransferFunction(dts);


    vector<double> num(numOrder+1),den(denOrder+1);
    Gz.GetZTransferFunction(num,den);
    SystemBlock idsys(num,den);

    for (double t=0; t<tmax; t+=dts)

    {
        in=1;//*(rand() % 10 + 1)-5;
        out=in > idsys;
        id.pushBack(out);
        //Gz.PrintZTransferFunction(dts);
        //Gz.PrintParamsVector();

    }

    id.Plot();
//    vector<double> params = Gz.GetParamsVector();
//    for (int i=0; i<params.size(); i++) cout << params[i] << endl;

//    p.Plot();

    return 0;
}
