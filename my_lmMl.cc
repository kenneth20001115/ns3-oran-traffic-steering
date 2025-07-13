#include "my_lmMl.h"
#include "oran-command-lte-2-lte-handover.h"
#include "oran-data-repository.h"
#include <ns3/abort.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>
#include <time.h>

#include<iostream>
#include <cfloat>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MyLmMl");

NS_OBJECT_ENSURE_REGISTERED(MyLmMl);

TypeId
MyLmMl::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::MyLmMl")
                            .SetParent<OranLm>()
                            .AddConstructor<MyLmMl>();

    return tid;
}

MyLmMl::MyLmMl(void)
    : OranLm()
{   
    NS_LOG_FUNCTION(this);
    runTime=0;
    vgd.setVar(0.05,0.01,0.1);
    M=0.05;
    throughputRun=0;
    throughputTotal=0;
    m_name = "MyLmMl";
     m_model = torch::jit::load("model.pt");
}

MyLmMl::~MyLmMl(void)
{
    NS_LOG_FUNCTION(this);
}

std::vector<Ptr<OranCommand>>
MyLmMl::Run(void)
{
    double start=clock();
    NS_LOG_FUNCTION(this);
    runTime++;
    
    std::vector<Ptr<OranCommand>> commands;
    double totalq=0;
    if (m_active)
    {
        NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                        "Attempting to run LM (" + m_name + ") with NULL Near-RT RIC");
        
        vgd.updateMb();
        M=vgd.getMb();
        std::cout<<"M: "<<M<<std::endl;;
        
        Ptr<OranDataRepository> data = m_nearRtRic->Data();
        std::vector<EnbInfo> enbInfos = GetEnbInfos(data);
        std::vector<UeInfo> ueInfos = GetUeInfos(data, enbInfos);
        commands = GetHandoverCommands(data, ueInfos, enbInfos, totalq);
        throughputRun+=GetThroughput(ueInfos);
        throughputTotal+=GetThroughput(ueInfos);
        totalloss += GetLoss(ueInfos);
        vgd.updateM(totalq);
        std::cout<<"total: "<<(throughputTotal*1500*8/1000)/(runTime*5+1)<<std::endl;
        std::cout<<"total loss: "<<totalloss/runTime<<std::endl;
    }
    double end=clock();
    std::cout<<"time: "<<(end-start)/CLOCKS_PER_SEC <<std::endl;
    times.push_back((end-start)/CLOCKS_PER_SEC);
    double tol=0.0;
    for(int i=0;i<times.size();i++)
    	tol += times[i];
    std::cout<<"average time: "<<tol/times.size()<<std::endl;
    // Return the commands.
    return commands;
}

std::vector<MyLmMl::UeInfo>
MyLmMl::GetUeInfos(Ptr<OranDataRepository> data, std::vector<MyLmMl::EnbInfo> enbinfos) const
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
                std::cout<<"ue "<<ueInfo.nodeId<<" loss :"<<ueInfo.loss<<" Tx: "<<ueInfo.Tx<<" Rx: "<<ueInfo.Rx<<" mcs: "<<static_cast<int>(ueInfo.mcs) <<std::endl;
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

std::vector<MyLmMl::EnbInfo>
MyLmMl::GetEnbInfos(Ptr<OranDataRepository> data) const
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

std::vector<Ptr<OranCommand>>
MyLmMl::GetHandoverCommands(
    Ptr<OranDataRepository> data,
    std::vector<MyLmMl::UeInfo> ueInfos,
    std::vector<MyLmMl::EnbInfo> enbInfos,
    double &tq)
{
    NS_LOG_FUNCTION(this << data);

    std::vector<Ptr<OranCommand>> commands;
    double myLoss=0;
    int rx=0;
    double totalq=0;
    int load[4]= {0};
	for(int i=0;i<ueInfos.size();i++){
		load[ueInfos[i].cellId-1]++;
	}
    for (auto ueInfo : ueInfos)
    {
        
         std::vector<float> inputv = {   (ueInfo.rsrp[0]+140)/80,
										 (ueInfo.rsrp[1]+140)/80.0,
										 (ueInfo.rsrp[2]+140)/80.0,
										 (ueInfo.rsrp[3]+140)/80.0,
										 float(load[0])*2/25.0,
										 float(load[1])*2/25.0,
										 float(load[2])*2/25.0,
										 float(load[3])*2/25.0,
										 float(ueInfo.sizetb)/2196.0};
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(torch::from_blob(inputv.data(), {1, 9}).to(torch::kFloat32));
        at::Tensor output = torch::softmax(m_model.forward(inputs).toTensor(), 1);
        int newCellId = output.argmax(1).item().toInt()+1;
        std::cout<<"newcellid"<<" "<<newCellId<<std::endl;
        int oldCellNodeId = 0;
        for (const auto& enbInfo : enbInfos)
        {
            // Check if this cell is the currently serving this UE.
            if (ueInfo.cellId == enbInfo.cellId)
            {
                // It is, so indicate record the ID of the cell that is
                // currently serving the UE.
                oldCellNodeId = enbInfo.nodeId;
            }
        }

        // Check if the ID of the closest cell is different from ID of the cell
        // that is currently serving the UE
        if (newCellId != ueInfo.cellId)
        {
            // It is, so issue a handover command.
            Ptr<OranCommandLte2LteHandover> handoverCommand =
                CreateObject<OranCommandLte2LteHandover>();
            // Send the command to the cell currently serving the UE.
            handoverCommand->SetAttribute("TargetE2NodeId", UintegerValue(oldCellNodeId));
            // Use the RNTI that the current cell is using to identify the UE.
            handoverCommand->SetAttribute("TargetRnti", UintegerValue(ueInfo.rnti));
            // Give the current cell the ID of the new cell to handover to.
            handoverCommand->SetAttribute("TargetCellId", UintegerValue(newCellId));
            // Log the command to the storage
            data->LogCommandLm(m_name, handoverCommand);
            // Add the command to send.
            commands.push_back(handoverCommand);

            LogLogicToRepository("eNB (CellID " + std::to_string(newCellId) + ")" +
                                 " is different than the currently attached eNB" + " (CellID " +
                                 std::to_string(ueInfo.cellId) + ")." +
                                 " Issuing handover command.");
            //load[ueInfo.cellId-1]--;
            //load[newCellId-1]++;
        }
        myLoss+=ueInfo.loss;
        rx+=ueInfo.Rx;
    }
    //std::cout<<"total loss: "<<myLoss/4.0<<std::endl;
    std::cout<<"total Rx: "<<rx<<std::endl;
    totalq /= ueInfos.size();
    tq=totalq;
    return commands;
}
double MyLmMl::GetThroughput(std::vector<MyLmMl::UeInfo> ueInfos) const{
        int totalRx=0;
        for (auto ueInfo : ueInfos)
            totalRx+=ueInfo.Rx;
        return totalRx;
}

double MyLmMl::GetLoss(std::vector<MyLmMl::UeInfo> ueInfos) const{
         double trx=0;
         double ttx=0;
        for (auto ueInfo : ueInfos){
            ttx+=ueInfo.Tx;
            trx+=ueInfo.Rx;
            }
        return (ttx-trx)/ttx;
}

} // namespace ns3

