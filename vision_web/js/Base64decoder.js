var keyStr = "ABCDEFGHIJKLMNOP" +
               "QRSTUVWXYZabcdef" +
               "ghijklmnopqrstuv" +
               "wxyz0123456789+/" +
               "=";
		
function decode64(input) {
   var output = "";
   var chr1, chr2, chr3 = "";
   var enc1, enc2, enc3, enc4 = "";
   var i = 0;

   // remove all characters that are not A-Z, a-z, 0-9, +, /, or =
   var base64test = /[^A-Za-z0-9\+\/\=]/g;
   if (base64test.exec(input)) {
      alert("There were invalid base64 characters in the input text.\n" +
            "Valid base64 characters are A-Z, a-z, 0-9, '+', '/',and '='\n" +
            "Expect errors in decoding.");
   }
   input = input.replace(/[^A-Za-z0-9\+\/\=]/g, "");

   do {
      enc1 = keyStr.indexOf(input.charAt(i++));
      enc2 = keyStr.indexOf(input.charAt(i++));
      enc3 = keyStr.indexOf(input.charAt(i++));
      enc4 = keyStr.indexOf(input.charAt(i++));

      chr1 = (enc1 << 2) | (enc2 >> 4);
      chr2 = ((enc2 & 15) << 4) | (enc3 >> 2);
      chr3 = ((enc3 & 3) << 6) | enc4;

      output = output + String.fromCharCode(chr1);

      if (enc3 != 64) {
         output = output + String.fromCharCode(chr2);
      }
      if (enc4 != 64) {
         output = output + String.fromCharCode(chr3);
      }

      chr1 = chr2 = chr3 = "";
      enc1 = enc2 = enc3 = enc4 = "";

   } while (i < input.length);
   
   var int = function(bits, input){
	    if(input < (1 << (bits-1))){
	        return input;
	    } else if (input >= (1 << bits)) {
	        throw "Error: input is not int"+bits+" format!!!";
	    } else {
	        return -((input ^ ((1 << bits)-1)) + 1);
	    }
	}
	
  return output;
  //return unescape(output);
}

decodeBase64 = function(s) {
  var e={},i,b=0,c,x,l=0,a,r='',w=String.fromCharCode,L=s.length;
  var A="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  for(i=0;i<64;i++){e[A.charAt(i)]=i;}
  for(x=0;x<L;x++){
      c=e[s.charAt(x)];
      b=(b<<6)+c;
      l+=6;
      while(l>=8){
      	((a=(b>>>(l-=8))&0xff)||(x<(L-2)))&&(r+=w(a));
      }
  }
  return r;
};