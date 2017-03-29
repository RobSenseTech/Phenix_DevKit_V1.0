// JavaScript Document

$(document).ready(function(){    
 
// TABLE STRIPING

    if ($('.overviewLCol table').length > 0) {
	    for (var i=0; i < $('.overviewLCol table').length; i++) {
			$('.overviewLCol table:eq(' + i + ') tr:odd').addClass('stripe');
		}
	}

if ($('#srch_results').length > 0) 
		{  // for registration pages
		for (var i=0; i < $('.overviewLCol table').length; i++) {
			$('.overviewLCol table:eq(' + i + ') tr:odd').removeClass('stripe');
		}
		$('#srch_results').addClass('featureTable');
		if ($('.adminSrchUser').length > 0)
		{
			$(".adminSrchUser").tablesorter({
 // pass the headers argument and assing a object 
              widgets: ['zebra']
			});
		}
		if ($('.adminProcess').length > 0)
		{
			$(".adminProcess").tablesorter({
 // pass the headers argument and assing a object 
		      widgets: ['zebra']
			});
		}
}

if ($('.sponsor-m-selectable-controls').length > 0) 
		{ // for registration sponsor admin pages
		alert('in sponsor-m-selectable-controls');
		$('.sponsor-m-selectable-controls').load('csi/dojo-button.htm');
		
		
		}
});