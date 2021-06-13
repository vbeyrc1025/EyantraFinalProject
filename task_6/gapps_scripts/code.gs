function doGet(e){
  
  var ss = SpreadsheetApp.getActive();

  var sheet = ss.getSheetByName(e.parameter["id"]);

  var headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];
  
  var cell = sheet.getRange('a1');
  var col = 0;
  var d = new Date();

  var lastRow = sheet.getLastRow();
  var lastColumn= sheet.getLastColumn();
  
 

   
  
  for (i in headers){

    // loop through the headers and if a parameter name matches the header name insert the value

    if (headers[i] == "Timestamp")
    {
      val = d.toDateString() + ", " + d.toLocaleTimeString();
    }
    else
    {
      val = e.parameter[headers[i]]; 
    }

    // append data to the last row
    cell.offset(lastRow, col).setValue(val);
    col++;
  }

  var latestrow = sheet.getLastRow();
  var latestcol = sheet.getLastColumn();

  
  var dstatus = sheet.getRange(latestrow,10).getValue();
 
  var sstatus = sheet.getRange(latestrow,10).getValue();
 

  if(e.parameter["id"]=="OrdersDispatched" && dstatus=="YES") {
  var orderid1 = sheet.getRange(latestrow,4).getValue();
  var city1    = sheet.getRange(latestrow,5).getValue();
  var quantity1= sheet.getRange(latestrow,8).getValue();
  var item1    = sheet.getRange(latestrow,6).getValue();
  var cost1    = sheet.getRange(latestrow,9).getValue();
  var disptime = sheet.getRange(latestrow,latestcol).getValue();
  var to = "eyrc.vb.0000@gmail.com";   //write your email id here
  var message1 = "Hello!"+"\n"+"\n"+"Your Order has been dispatched.contact us if you have any questions.We are here to help you."+"\n"+"ORDER SUMMERY: "+"\n"+"\n"+"Order Number: " + orderid1+ " \n"+"Item: "+item1+"\n"+"Quantitiy: "+quantity1+"\n"+"Dispatch Date and Time: "+disptime+"\n"+"City: "+city1+"\n"+"Cost: "+cost1+"\n";
                  

    MailApp.sendEmail(to, "Order Dispatched!", message1);
  }
  if(e.parameter["id"]=="OrdersShipped" && sstatus=="YES") {
    var orderid2 = sheet.getRange(latestrow,4).getValue();
    var city2    = sheet.getRange(latestrow,5).getValue();
    var quantity2= sheet.getRange(latestrow,8).getValue();
    var item2    = sheet.getRange(latestrow,6).getValue();
    var cost2    = sheet.getRange(latestrow,9).getValue();
    var shiptime= sheet.getRange(latestrow,11).getValue();
    var estimtime=  sheet.getRange(latestrow,latestcol).getValue();
    var to = "eyrc.vb.0000@gmail.com";   //write your email id here
    if(item2 == "Medicine"){
      var days = 1;
    }
    if(item2 == "Food"){
      var days = 3;
    }
    if(item2 == "Clothes"){
      var days = 5;
    }
    var message2 = "Hello!"+"\n"+"\n"+"Your Order has been shipped.It will be drone deliverd to you in "+days+" days.Contact us if you have any questions.We are here to help you."+"\n"+"\n"+"ORDER SUMMERY: "+"\n"+"\n"+"Order Number: " + orderid2+ " \n"+"Item: "+item2+"\n"+"Quantitiy: "+quantity2+"\n"+"Shipping Date and Time: "+shiptime+"\n"+"City: "+city2+"\n"+"Cost: "+cost2+"\n"+"Estimated time of Delivery: "+ estimtime;                

    MailApp.sendEmail(to, "Order Shipped", message2);
  }

  return ContentService.createTextOutput('success');

  


}