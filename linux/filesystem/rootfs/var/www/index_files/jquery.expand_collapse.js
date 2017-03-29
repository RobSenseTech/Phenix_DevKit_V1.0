
// EXPAND AND COLLAPSE FUNCTIONALITY

$(document).ready(function(){ 
						   
// --------------------------------------------------------
// EXPAND ALL / COLLAPSE ALL
// --------------------------------------------------------
	$('.showAll').click( function() {
		$(this).parent().siblings('div.ec').children('div.ec-content').fadeIn('fast');
		$(this).parent().siblings('div.ec').children('a.eclink').addClass('collapse').removeClass('expand');
		return false;
	});
	$('.hideAll').click( function() {
		$(this).parent().siblings('div.ec').children('div.ec-content').fadeOut('fast');
		$(this).parent().siblings('div.ec').children('a.eclink').addClass('expand').removeClass('collapse');
		return false;
	});

// --------------------------------------------------------
// EXPAND AND COLLAPSE INDIVIDUALLY
// --------------------------------------------------------

    $('.eclink')
	    // add class="expand" to all links
		.addClass('expand')
		
		// H B X: add name attribute to all expand / collapse links
		.each(function() {
			var eclinkText = $(this).text();		
		})
		
		// changing class styles on both hover on and hover off
		.hover(
		function () {
        	$(this).parent().addClass('echover');
     	},
      	function () {
        	$(this).parent().removeClass('echover');
      	})
		
		// functionality of expanding or collapsing hidden div based on current class
		.click(
			function () {
				$(this).parent().removeClass('ecbackground');
				var classValue = $(this).attr('class'); // grab class value
				var eclinkText = $(this).text(); // grab link text
				var classExpand = /expand/; 
				var classCollapse = /collapse/;
				if (classExpand.test(classValue) == true) //if class value contains expand then show hidden content
					{
					$(this).removeClass('expand').addClass('collapse').parent().children('div.ec-content').fadeIn('fast');
					return false;
					}
				else  // if class doesn't contain expand, then hide visible content
					{
					$(this).removeClass('collapse').addClass('expand').parent().children('div.ec-content').fadeOut('fast');
					$(this).parent().addClass('ecbackground');
					return false;
					}
		});
     $('.ec:last').addClass('ecBorderBot');
  });
