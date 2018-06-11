# CL2-2017.2-Reconfigurable-System-for-DC-Motor-Controlling

module MOTOR(
	input Clock,
	output reg [0:6]IA,
	output reg [0:6]IA2,
	output reg [0:6]IF,
	output reg [0:6]IF2,
	output reg [0:6]WR,
	output reg [0:6]WR2,
	output reg [0:6]WR3,
	output reg [0:6]WR4
);			

	
	parameter Tm=0; //Sem carga
	reg [0:31]Ia[0:6];
	reg [0:31]If[0:6];
	reg [0:31]Wr[0:6]; 

	reg [0:31]Va[0:6];
	reg [0:31]Erro[0:6];
	reg Cont;
	reg CLK = 1'b0;
	
	reg [0:3]RestIf;
	reg [0:3]RestIf2;
	reg [0:3]RestIa;
	reg [0:3]RestIa2;
	reg [0:3]RestWr;
	reg [0:3]RestWr2;
	reg [0:3]RestWr3;
	reg [0:3]RestWr4;
	
	
	reg [0:7]auxt = 8'd1;
	reg auxErro;

	//Tentativa de tirar dos numeros decimais, ja que o Quartus nao permite
	parameter Ra = 8000000; 		 //0.08
	parameter La = 100;     		 //0.0001
	parameter Rf = 50000000;		 //0.5
	parameter Lf = 1000;    		 //0.001
	parameter Vf = 100000000;		 //1
	parameter Jm = 1000;     		 //0.001
	parameter Fm = 100;	          //0.0001
	parameter Km = 47616;          //0.47616
	parameter Kp = -233900;        //-0.2339
	parameter Ki = 47616400;       //47.6164
	parameter Kd = 12;             //0.0012
	parameter WRef = 125;
	parameter h = 1;               //0.000001
	
	
	//Deixando a funcoes que sao constantes, ja calculadas
	parameter Rf_Lf=500000000;     //(Rf/Lf)*1000000
	parameter Vf_Lf=1000000000;    //(Vf/LF)*1000000
	parameter Ra_La=800000000;     //(Ra/La)*1000000
	parameter Fm_Jm=10000;         //(Fm/Jm)*100000
	parameter Km_Jm=4761600000;    //(Km/Jm)*1000000
	parameter Km_La = 47616000000; //(Km/La)*1000000
	parameter Tm_Jm=0;
	
	parameter D = 1200000000;      //(Kd/h)*1000000;
	parameter PD = -2399766100;    //(-Kp - 2*Kd/h)*1000000
	parameter PID = 1199766148;    //1000000*(Kp + Ki*(h) + Kd/(h));
	
	/*always@(Clock) begin 
		Cont= Cont+1;
		if(Cont==50) begin
			CLK = ~CLK;
			Cont = 0;
		end
	end*/
	
	reg flag=1'b0;
	integer j;
	
	reg [0:7]t;

	
	
	always@(t) begin 
		auxt= auxt+1;
		
		if(t== 8'd6)
			t=8'd2;		//Sempre volta para dois, pois havera equacoes que calcula "Erro[t-2]" 
		
		if(t== 8'd4) begin 
			auxErro<=Erro[t];
		end
		if(t== 8'd5) begin 
			Erro[0]=auxErro;
			Erro[1]=Erro[t];
			Va[1] = Va[t];
			If[1] = If[t];
			Ia[1] = Ia[t];
			Wr[1] = Wr[t];
		end
		if(flag==0) begin  //iniciar
		
			for(j=0;j<7;j=j+1)begin
				If[j]   = 32'd2;
				Ia[j]   = 32'd0;
				Va[j]   = 32'd0;
				Wr[j]   = 32'd125;
				Erro[j] = 32'd125;
			end
			flag=1'b1;     //impede que entre no if novamente
		end
		if(t>1) begin
			/*Erro[t] = WRef - Wr[t-1];
			Va[t] = (Erro[t] *(PID) + Erro[t-1]*PD + Erro[t-2]*D + Va[t-1]);
			//Va[2:5]
			If[t] = (If[t-1] + ((-Rf/Lf)*If[t-1] + Vf/Lf)*h);
			Ia[t] = (Ia[t-1] + ((-Ra/La)*Ia[t-1] + Va[t]/La - (Km/La)*If[t-1]*Wr[t-1]) *h);
			Wr[t] = (Wr[t-1] + ((-Fm/Jm)*Wr[t-1] + (Km/Jm)*If[t-1]*Ia[t-1] - Tm/Jm) *h);*/
		

			If[t]    =   (If[t-1] + ((- Rf_Lf)*If[t-1] + Vf_Lf)*h);
			Ia[t]   =   (Ia[t-1] + ((- Ra_La)*Ia[t-1]*10 + (Va[t-1]/La)/1000000 - (Km_La)*If[t-1]*Wr[t-1])*h); //Km_La * 10
			Wr[t]   =   (Wr[t-1] + ((- Fm_Jm)*Wr[t-1] + (Km_Jm)*If[t-1]*Ia[t-1] - Tm_Jm) *h);					   //Fm_Jm *1000 e Km_Jm*1000
			Erro[t] =   (WRef - Wr[t]);
			Va[t]   =   (Va[t-1] + Erro[t] *(PID) + Erro[t-1]*PD + Erro[t-2]*D );
		end
		RestIf = If[t]/10;
		RestIf2 = If[t]%10;
		RestIa = Ia[t]/10;
		RestIa2 = Ia[t]%10;
		RestWr = Wr[t]/1000;
		RestWr2 = (Wr[t]%1000)/100;
		RestWr3 = ((Wr[t]%1000)%100)/10;
		RestWr4 = ((Wr[t]%1000)%100)%10;
		
		case(RestIf2)  		//Display 7 segmentos
		4'b0000: IF2 = 7'b0000001;
		4'b0001: IF2 = 7'b1001111;
		4'b0010: IF2 = 7'b0010010;
		4'b0011: IF2 = 7'b0000110;
		4'b0100: IF2 = 7'b1001100;
		4'b0101: IF2 = 7'b0100100;
		4'b0110: IF2 = 7'b0100000;
		4'b0111: IF2 = 7'b0001111;
		4'b1000: IF2 = 7'b0000000;
		4'b1001: IF2 = 7'b0000100;
		endcase
		case(RestIf)
		4'b0000: IF = 7'b0000001;
		4'b0001: IF = 7'b1001111;
		4'b0010: IF = 7'b0010010;
		4'b0011: IF = 7'b0000110;
		4'b0100: IF = 7'b1001100;
		4'b0101: IF = 7'b0100100;
		4'b0110: IF = 7'b0100000;
		4'b0111: IF = 7'b0001111;
		4'b1000: IF = 7'b0000000;
		4'b1001: IF = 7'b0000100;
		endcase
		case(RestIa)
		4'b0000: IA = 7'b0000001;
		4'b0001: IA = 7'b1001111;
		4'b0010: IA = 7'b0010010;
		4'b0011: IA = 7'b0000110;
		4'b0100: IA = 7'b1001100;
		4'b0101: IA = 7'b0100100;
		4'b0110: IA = 7'b0100000;
		4'b0111: IA = 7'b0001111;
		4'b1000: IA = 7'b0000000;
		4'b1001: IA = 7'b0000100;
		endcase
		case(RestIa2)
		4'b0000: IA2 = 7'b0000001;
		4'b0001: IA2 = 7'b1001111;
		4'b0010: IA2 = 7'b0010010;
		4'b0011: IA2 = 7'b0000110;
		4'b0100: IA2 = 7'b1001100;
		4'b0101: IA2 = 7'b0100100;
		4'b0110: IA2 = 7'b0100000;
		4'b0111: IA2 = 7'b0001111;
		4'b1000: IA2 = 7'b0000000;
		4'b1001: IA2 = 7'b0000100;
		endcase
		case(RestWr)
		4'b0000: WR = 7'b0000001;
		4'b0001: WR = 7'b1001111;
		4'b0010: WR = 7'b0010010;
		4'b0011: WR = 7'b0000110;
		4'b0100: WR = 7'b1001100;
		4'b0101: WR = 7'b0100100;
		4'b0110: WR = 7'b0100000;
		4'b0111: WR = 7'b0001111;
		4'b1000: WR = 7'b0000000;
		4'b1001: WR = 7'b0000100;
		endcase
		case(RestWr2)
		4'b0000: WR2 = 7'b0000001;
		4'b0001: WR2 = 7'b1001111;
		4'b0010: WR2 = 7'b0010010;
		4'b0011: WR2 = 7'b0000110;
		4'b0100: WR2 = 7'b1001100;
		4'b0101: WR2 = 7'b0100100;
		4'b0110: WR2 = 7'b0100000;
		4'b0111: WR2 = 7'b0001111;
		4'b1000: WR2 = 7'b0000000;
		4'b1001: WR2 = 7'b0000100;
		endcase
		case(RestWr3)
		4'b0000: WR3 = 7'b0000001;
		4'b0001: WR3 = 7'b1001111;
		4'b0010: WR3 = 7'b0010010;
		4'b0011: WR3 = 7'b0000110;
		4'b0100: WR3 = 7'b1001100;
		4'b0101: WR3 = 7'b0100100;
		4'b0110: WR3 = 7'b0100000;
		4'b0111: WR3 = 7'b0001111;
		4'b1000: WR3 = 7'b0000000;
		4'b1001: WR3 = 7'b0000100;
		endcase
		case(RestWr4)
		4'b0000: WR4 = 7'b0000001;
		4'b0001: WR4 = 7'b1001111;
		4'b0010: WR4 = 7'b0010010;
		4'b0011: WR4 = 7'b0000110;
		4'b0100: WR4 = 7'b1001100;
		4'b0101: WR4 = 7'b0100100;
		4'b0110: WR4 = 7'b0100000;
		4'b0111: WR4 = 7'b0001111;
		4'b1000: WR4 = 7'b0000000;
		4'b1001: WR4 = 7'b0000100;
		endcase
	end 
  
endmodule 
