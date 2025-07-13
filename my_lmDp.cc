#include "my_lmDp.h"
#include "oran-command-lte-2-lte-handover.h"
#include "oran-data-repository.h"
#include <ns3/abort.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>
#include <cmath>
#include<iostream>
#include <cfloat>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MyLmDp");

NS_OBJECT_ENSURE_REGISTERED(MyLmDp);

TypeId
MyLmDp::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::MyLmDp")
                            .SetParent<OranLm>()
                            .AddConstructor<MyLmDp>();

    return tid;
}

MyLmDp::MyLmDp(void)
    : OranLm()
{   
    NS_LOG_FUNCTION(this);
    runTime=0;
    M=0.05;
    throughputRun=0;
    throughputTotal=0;
    m_name = "MyLmDp";
    run=0;
    d=2;
}

MyLmDp::~MyLmDp(void)
{
    NS_LOG_FUNCTION(this);
}

std::vector<Ptr<OranCommand>>
MyLmDp::Run(void)
{
    NS_LOG_FUNCTION(this);
    runTime++;
    
    std::vector<Ptr<OranCommand>> commands;
    double totalq=0;
    if (m_active)
    {
        NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                        "Attempting to run LM (" + m_name + ") with NULL Near-RT RIC");
        
        
        Ptr<OranDataRepository> data = m_nearRtRic->Data();
        std::vector<EnbInfo> enbInfos = GetEnbInfos(data);
        std::vector<UeInfo> ueInfos = GetUeInfos(data, enbInfos);
        std::vector<uint16_t> a(ueInfos.size());
        if(!run){
		    commands = GetHandoverCommands(ueInfos,enbInfos,data,a);
		    throughputRun+=GetThroughput(ueInfos);
		    throughputTotal+=GetThroughput(ueInfos);
		    totalloss += GetLoss(ueInfos);
		    std::cout<<"total: "<<(throughputTotal*1500*8/1000)/(runTime*5+1)<<std::endl;
		    std::cout<<"total loss: "<<totalloss/runTime<<std::endl;
		    saveData(ueInfos, a);
		    //run=1;
        }
    }
    
    // Return the commands.
    return commands;
}

void MyLmDp::saveData(std::vector<UeInfo> ueInfos,std::vector<uint16_t> a){
	std::ofstream outfile;
	outfile.open("test.csv",std::ios::app);
	int load[4]= {0};
	for(int i=0;i<ueInfos.size();i++){
		load[ueInfos[i].cellId-1]++;
	}
	for(int i=0;i<ueInfos.size();i++){
	        outfile<<ueInfos[i].cellId<<",";
		for(int j=0;j<ueInfos[i].rsrp.size();j++){
			outfile<<ueInfos[i].rsrp[j]<<",";
		}
		for(int j=0;j<4;j++){
			outfile<<load[j]<<",";
		}
		outfile<< static_cast<int>(ueInfos[i].mcs)<< "," <<ueInfos[i].sizetb<<","<<a[i]<<"\n";
	}
	outfile.close();
}

std::vector<MyLmDp::UeInfo>
MyLmDp::GetUeInfos(Ptr<OranDataRepository> data, std::vector<MyLmDp::EnbInfo> enbinfos) const
{
    NS_LOG_FUNCTION(this << data);

    std::vector<UeInfo> ueInfos;
    for (auto ueId : data->GetLteUeE2NodeIds())
    {
        UeInfo ueInfo;
        ueInfo.nodeId = ueId;
        // Get the current cell ID and RNTI of the UE and record it.
        bool found;
        std::tie(found, ueInfo.cellId, ueInfo.rnti) = data->GetLteUeCellInfo(ueInfo.nodeId);
        if (found)
        {
            // Get the latest location of the UE.
            std::map<Time, Vector> nodePositions =
                data->GetNodePositions(ueInfo.nodeId, Seconds(0), Simulator::Now());

            if (!nodePositions.empty())
            {
                // We found both the cell and location informtaion for this UE
                // so record it for a later analysis.
                ueInfo.position = nodePositions.rbegin()->second;
                ueInfo.loss = data->GetAppLoss(ueInfo.nodeId);
                ueInfo.Rx = data->GetRx(ueInfo.nodeId);
                ueInfo.Tx = data->GetTx(ueInfo.nodeId);
                uint64_t id;
                ueInfo.rsrp.resize(4);
                auto rsrpMeasurements = data->GetLteUeRsrpRsrq(ueInfo.nodeId);
				for (auto rsrpMeasurement : rsrpMeasurements)
				{
				    uint16_t rnti;
				    uint16_t cellId;
				    double rsrp;
				    double rsrq;
				    bool isServingCell;
				    uint16_t componentCarrierId;
				    std::tie(rnti, cellId, rsrp, rsrq, isServingCell, componentCarrierId) = rsrpMeasurement;
				    ueInfo.rsrp[cellId-1] = rsrp;
				    //std::cout<<"ue: "<<ueInfo.nodeId<<" cellId: "<<cellId<<" rsrp: "<<rsrp<<std::endl;
				 }
                for(int i=0;i<enbinfos.size();i++)
                	if(enbinfos[i].cellId==ueInfo.cellId)
                		id=enbinfos[i].nodeId;

                ueInfo.mcs = data->Getmcs(id, ueInfo.rnti);
                ueInfo.sizetb = data->Getsizetb(id, ueInfo.rnti);
                std::cout<<"ue "<<ueInfo.nodeId<<" loss :"<<ueInfo.loss<<" Tx: "<<ueInfo.Tx<<" Rx: "<<ueInfo.Rx<<" mcs: "<< static_cast<int>(ueInfo.mcs) <<std::endl;
                ueInfos.push_back(ueInfo);
            }
            else
            {
                NS_LOG_INFO("Could not find LTE UE location for E2 Node ID = " << ueInfo.nodeId);
            }
        }
        else
        {
            NS_LOG_INFO("Could not find LTE UE cell info for E2 Node ID = " << ueInfo.nodeId);
        }
    }
    return ueInfos;
}

std::vector<MyLmDp::EnbInfo>
MyLmDp::GetEnbInfos(Ptr<OranDataRepository> data) const
{
    NS_LOG_FUNCTION(this << data);

    std::vector<EnbInfo> enbInfos;
    for (auto enbId : data->GetLteEnbE2NodeIds())
    {
        EnbInfo enbInfo;
        enbInfo.nodeId = enbId;
        // Get the cell ID of this eNB and record it.
        bool found;
        std::tie(found, enbInfo.cellId) = data->GetLteEnbCellInfo(enbInfo.nodeId);
        if (found)
        {
            // Get all known locations of the eNB.
            std::map<Time, Vector> nodePositions =
                data->GetNodePositions(enbInfo.nodeId, Seconds(0), Simulator::Now());

            if (!nodePositions.empty())
            {
                // We found both the cell and location information for this
                // eNB so record it for a later analysis.
                enbInfo.position = nodePositions.rbegin()->second;
                enbInfos.push_back(enbInfo);
            }
            else
            {
                NS_LOG_INFO("Could not find LTE eNB location for E2 Node ID = " << enbInfo.nodeId);
            }
        }
        else
        {
            NS_LOG_INFO("Could not find LTE eNB cell info for E2 Node ID = " << enbInfo.nodeId);
        }
    }
    return enbInfos;
}

double MyLmDp::value(double rsrp, int load){
	double v=abr[0]*(rsrp+140.0)/(80.0)+std::log2(1-double(load)*d/25.0)*0.5;
	return v;
}

double MyLmDp::solve(std::vector<MyLmDp::UeInfo> &ueInfos, int j,int c1,int c2,int c3,int c4, int l1,int l2,int l3,int l4){
	if(j==ueInfos.size()) return 0;
	if(dp[j][c1][c2][c3][c4]!=0 ) return dp[j][c1][c2][c3][c4];
	
	double best = -DBL_MAX;
	uint16_t a = 0;
	if(c1>d){
		double v=value(ueInfos[j].rsrp[0], l1)+solve(ueInfos,j+1,c1-d,c2,c3,c4,l1+1,l2,l3,l4);
		if(v>best){
			best=v;
			a=1;
		}
	}
	if(c2>d){
		double v=value(ueInfos[j].rsrp[1], l2)+solve(ueInfos,j+1,c1,c2-d,c3,c4,l1,l2+1,l3,l4);
		if(v>best){
			best=v;
			a=2;
		}
	}
	if(c3>d){
		double v=value(ueInfos[j].rsrp[2], l3)+solve(ueInfos,j+1,c1,c2,c3-d,c4,l1,l2,l3+1,l4);
		if(v>best){
			best=v;
			a=3;
		}
	}
	if(c4>d){
		double v=value(ueInfos[j].rsrp[3], l4)+solve(ueInfos,j+1,c1,c2,c3,c4-d,l1,l2,l3,l4+1);
		if(v>best){
			best=v;
			a=4;
		}
	}
	dp[j][c1][c2][c3][c4]=best;
	assign[j][c1][c2][c3][c4]=a;
	return best;
}

std::vector<Ptr<OranCommand>>
MyLmDp::GetHandoverCommands(
	std::vector<MyLmDp::UeInfo> ueInfos,
    std::vector<MyLmDp::EnbInfo> enbInfos,
    Ptr<OranDataRepository> data,
    std::vector<uint16_t> &a)
{
    NS_LOG_FUNCTION(this << data);

    std::vector<Ptr<OranCommand>> commands;
    memset(dp,0,sizeof(dp));
    memset(assign,0,sizeof(assign));
    solve(ueInfos,0,25,25,25,25,0,0,0,0);
    int c1=25,c2=25,c3=25,c4=25;
    for (int i=0;i<ueInfos.size();i++){
    	a[i]=assign[i][c1][c2][c3][c4];
    	std::cout<<dp[i][c1][c2][c3][c4]<<" "<<assign[i][c1][c2][c3][c4]<<std::endl;
    	switch(assign[i][c1][c2][c3][c4]){
    		case 1:
    			c1-=d;
    			break;
    		case 2:
    			c2-=d;
    			break;
    		case 3:
    			c3-=d;
    			break;
    		case 4:
    			c4-=d;
    			break;
    		default:
    			std::cout<<"error!!!\n";
    	}
    }
    std::cout<<"\n";
    for (auto i=0;i<ueInfos.size();i++)
    {
        
       
        // Check if the ID of the closest cell is different from ID of the cell
        // that is currently serving the UE
        uint16_t newCellId=a[i];
        uint64_t oldCellNodeId;
        std::cout<<a[i]<<" ";
        
        for (const auto& enbInfo : enbInfos)
        {
            // Check if this cell is the currently serving this UE.
            if (ueInfos[i].cellId == enbInfo.cellId)
            {
                // It is, so indicate record the ID of the cell that is
                // currently serving the UE.
                oldCellNodeId = enbInfo.nodeId;
            }
        }
        
        if (newCellId != ueInfos[i].cellId)
        {
            std::cout<<"cellId: "<<ueInfos[i].cellId<<" rnti: "<<ueInfos[i].rnti<<std::endl;
            // It is, so issue a handover command.
            Ptr<OranCommandLte2LteHandover> handoverCommand =
                CreateObject<OranCommandLte2LteHandover>();
            // Send the command to the cell currently serving the UE.
            handoverCommand->SetAttribute("TargetE2NodeId", UintegerValue(oldCellNodeId));
            // Use the RNTI that the current cell is using to identify the UE.
            handoverCommand->SetAttribute("TargetRnti", UintegerValue(ueInfos[i].rnti));
            // Give the current cell the ID of the new cell to handover to.
            handoverCommand->SetAttribute("TargetCellId", UintegerValue(newCellId));
            // Log the command to the storage
            data->LogCommandLm(m_name, handoverCommand);
            // Add the command to send.
            commands.push_back(handoverCommand);

            LogLogicToRepository("eNB (CellID " + std::to_string(newCellId) + ")" +
                                 " is different than the currently attached eNB" + " (CellID " +
                                 std::to_string(ueInfos[i].cellId) + ")." +
                                 " Issuing handover command.");
        }
    }
    std::cout<<"\n"<<commands.size()<<std::endl;
    //std::cout<<"total loss: "<<myLoss/4.0<<std::endl;
    return commands;
}
double MyLmDp::GetThroughput(std::vector<MyLmDp::UeInfo> ueInfos) const{
        int totalRx=0;
        for (auto ueInfo : ueInfos)
            totalRx+=ueInfo.Rx;
        return totalRx;
}

double MyLmDp::GetLoss(std::vector<MyLmDp::UeInfo> ueInfos) const{
         double trx=0;
         double ttx=0;
        for (auto ueInfo : ueInfos){
            ttx+=ueInfo.Tx;
            trx+=ueInfo.Rx;
            }
        return (ttx-trx)/ttx;
}

} // namespace ns3

